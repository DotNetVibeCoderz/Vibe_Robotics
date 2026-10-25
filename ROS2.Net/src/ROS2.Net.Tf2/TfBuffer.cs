using System.Collections.Concurrent;
using System.Globalization;
using System.Text;
using ROS2.Net.Msgs.GeometryMsgs;

namespace ROS2.Net.Tf2;

/// <summary>
/// Stores the history of every coordinate frame and answers "where is frame A relative to frame B at
/// time t", like <c>tf2::BufferCore</c> / <c>tf2_ros::Buffer</c>. Thread-safe: each frame has its own
/// lock, so inserts from a listener and lookups from control loops rarely contend.
/// </summary>
public sealed class TfBuffer
{
    private const int MaxGraphDepth = 1000;

    private readonly ConcurrentDictionary<string, int> _ids = new(StringComparer.Ordinal);
    private readonly object _registryGate = new();
    private TimeCache?[] _caches = new TimeCache?[64];
    private string[] _names = new string[64];
    private int _count;
    private readonly long _cacheNs;
    private TaskCompletionSource _changed = new(TaskCreationOptions.RunContinuationsAsynchronously);

    /// <param name="cacheTime">How much history to keep per frame (tf2 default: 10 s).</param>
    public TfBuffer(TimeSpan? cacheTime = null)
    {
        CacheTime = cacheTime ?? TimeSpan.FromSeconds(10);
        _cacheNs = CacheTime.Ticks * 100;
    }

    public TimeSpan CacheTime { get; }

    /// <summary>Raised after every inserted transform (on the inserting thread).</summary>
    public event Action<TransformStamped>? TransformAdded;

    // ------------------------------------------------------------------ insertion

    /// <summary>Adds a transform. Returns false if it was rejected (invalid or older than the cache window).</summary>
    public bool SetTransform(TransformStamped transform, string authority = "default_authority", bool isStatic = false)
    {
        var parent = StripSlash(transform.Header.FrameId);
        var child = StripSlash(transform.ChildFrameId);
        if (parent.Length == 0 || child.Length == 0)
        {
            throw new InvalidTransformArgumentException("Transform frame ids must not be empty.");
        }

        if (parent == child)
        {
            throw new InvalidTransformArgumentException($"Transform from '{parent}' to itself is not allowed.");
        }

        var t = transform.Transform;
        if (double.IsNaN(t.Translation.X + t.Translation.Y + t.Translation.Z + t.Rotation.X + t.Rotation.Y + t.Rotation.Z + t.Rotation.W))
        {
            throw new InvalidTransformArgumentException($"Transform {parent} -> {child} contains NaN.");
        }

        var tf = Transform3d.FromMsg(t);
        tf = tf with { Rotation = tf.Rotation.Normalized() };
        var stamp = RosTime.FromMsg(transform.Header.Stamp).Nanoseconds;

        var parentId = GetOrAddId(parent);
        var childId = GetOrAddId(child);
        var cache = GetOrAddCache(childId, child, isStatic);
        cache.Authority = authority;
        var ok = cache.Insert(new TfSample(stamp, parentId, tf), isStatic);
        if (ok)
        {
            Interlocked.Exchange(ref _changed, new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously)).TrySetResult();
            TransformAdded?.Invoke(transform);
        }

        return ok;
    }

    /// <summary>Convenience overload: adds <c>parent → child</c> at <paramref name="stamp"/>.</summary>
    public bool SetTransform(string parent, string child, Transform3d transform, RosTime stamp, bool isStatic = false, string authority = "default_authority")
    {
        var msg = new TransformStamped { ChildFrameId = child, Transform = transform.ToMsg() };
        msg.Header.FrameId = parent;
        stamp.CopyTo(msg.Header.Stamp);
        return SetTransform(msg, authority, isStatic);
    }

    /// <summary>Removes all data.</summary>
    public void Clear()
    {
        lock (_registryGate)
        {
            _ids.Clear();
            _caches = new TimeCache?[64];
            _names = new string[64];
            _count = 0;
        }
    }

    private int GetOrAddId(string name)
    {
        if (_ids.TryGetValue(name, out var id))
        {
            return id;
        }

        lock (_registryGate)
        {
            if (_ids.TryGetValue(name, out id))
            {
                return id;
            }

            id = _count++;
            if (id >= _names.Length)
            {
                Array.Resize(ref _names, _names.Length * 2);
                Array.Resize(ref _caches, _caches.Length * 2);
            }

            _names[id] = name;
            _ids[name] = id;
            return id;
        }
    }

    private TimeCache GetOrAddCache(int id, string name, bool isStatic)
    {
        var existing = Volatile.Read(ref _caches)[id];
        if (existing is not null)
        {
            return existing;
        }

        lock (_registryGate)
        {
            return _caches[id] ??= new TimeCache(name, id, _cacheNs, isStatic);
        }
    }

    private TimeCache? CacheOf(int id)
    {
        var caches = Volatile.Read(ref _caches);
        return id < caches.Length ? caches[id] : null;
    }

    private static string StripSlash(string frame) => frame.Length > 0 && frame[0] == '/' ? frame[1..] : frame;

    // ------------------------------------------------------------------ lookups

    /// <summary>
    /// The transform that maps data in <paramref name="sourceFrame"/> into <paramref name="targetFrame"/>
    /// at <paramref name="time"/>. <see cref="RosTime.Zero"/> means the latest time all links share.
    /// </summary>
    /// <exception cref="TransformException">The lookup failed (unknown frame, disconnected, or extrapolation).</exception>
    public TransformStamped LookupTransform(string targetFrame, string sourceFrame, RosTime time)
    {
        var (tf, stamp) = Lookup(targetFrame, sourceFrame, time.Nanoseconds);
        return ToStamped(targetFrame, sourceFrame, tf, stamp);
    }

    /// <summary>Time-travel lookup: <paramref name="sourceFrame"/> at <paramref name="sourceTime"/> into <paramref name="targetFrame"/> at <paramref name="targetTime"/>, through a frame fixed in the world.</summary>
    public TransformStamped LookupTransform(string targetFrame, RosTime targetTime, string sourceFrame, RosTime sourceTime, string fixedFrame)
    {
        var (targetFromFixed, tStamp) = Lookup(targetFrame, fixedFrame, targetTime.Nanoseconds);
        var (fixedFromSource, _) = Lookup(fixedFrame, sourceFrame, sourceTime.Nanoseconds);
        return ToStamped(targetFrame, sourceFrame, targetFromFixed * fixedFromSource, tStamp);
    }

    /// <summary>Like <see cref="LookupTransform(string, string, RosTime)"/> but returns the math type.</summary>
    public Transform3d LookupTransform3d(string targetFrame, string sourceFrame, RosTime time) =>
        Lookup(targetFrame, sourceFrame, time.Nanoseconds).Transform;

    /// <summary>Non-throwing lookup.</summary>
    public bool TryLookupTransform(string targetFrame, string sourceFrame, RosTime time, out Transform3d transform, out string? error)
    {
        var ok = TryLookup(StripSlash(targetFrame), StripSlash(sourceFrame), time.Nanoseconds, out transform, out _, out error, out _);
        return ok;
    }

    /// <summary>Whether <see cref="LookupTransform(string, string, RosTime)"/> would succeed.</summary>
    public bool CanTransform(string targetFrame, string sourceFrame, RosTime time) =>
        TryLookupTransform(targetFrame, sourceFrame, time, out _, out _);

    public bool CanTransform(string targetFrame, RosTime targetTime, string sourceFrame, RosTime sourceTime, string fixedFrame) =>
        CanTransform(targetFrame, fixedFrame, targetTime) && CanTransform(fixedFrame, sourceFrame, sourceTime);

    /// <summary>Waits until the transform becomes available, then returns it.</summary>
    /// <exception cref="TransformTimeoutException">No transform arrived within <paramref name="timeout"/>.</exception>
    public async Task<TransformStamped> WaitForTransformAsync(string targetFrame, string sourceFrame, RosTime time, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        var deadline = Environment.TickCount64 + (long)timeout.TotalMilliseconds;
        while (true)
        {
            var signal = Volatile.Read(ref _changed).Task;
            if (CanTransform(targetFrame, sourceFrame, time))
            {
                return LookupTransform(targetFrame, sourceFrame, time);
            }

            var remaining = deadline - Environment.TickCount64;
            if (remaining <= 0)
            {
                TryLookupTransform(targetFrame, sourceFrame, time, out _, out var error);
                throw new TransformTimeoutException($"Timed out after {timeout.TotalSeconds:0.###} s waiting for {sourceFrame} -> {targetFrame}: {error}");
            }

            await Task.WhenAny(signal, Task.Delay(TimeSpan.FromMilliseconds(Math.Min(remaining, 100)), cancellationToken)).ConfigureAwait(false);
            cancellationToken.ThrowIfCancellationRequested();
        }
    }

    private (Transform3d Transform, long Stamp) Lookup(string target, string source, long timeNs)
    {
        target = StripSlash(target);
        source = StripSlash(source);
        if (TryLookup(target, source, timeNs, out var tf, out var stamp, out var error, out var kind))
        {
            return (tf, stamp);
        }

        throw kind switch
        {
            ErrorKind.Lookup => new LookupException(error!),
            ErrorKind.Connectivity => new ConnectivityException(error!),
            ErrorKind.InvalidArgument => new InvalidTransformArgumentException(error!),
            _ => new ExtrapolationException(error!),
        };
    }

    private enum ErrorKind
    {
        None,
        Lookup,
        Connectivity,
        Extrapolation,
        InvalidArgument,
    }

    private bool TryLookup(string target, string source, long timeNs, out Transform3d result, out long stamp, out string? error, out ErrorKind kind)
    {
        result = Transform3d.Identity;
        stamp = timeNs;
        error = null;
        kind = ErrorKind.None;
        if (target.Length == 0 || source.Length == 0)
        {
            error = "Frame ids must not be empty.";
            kind = ErrorKind.InvalidArgument;
            return false;
        }

        if (!_ids.TryGetValue(source, out var sourceId))
        {
            error = $"\"{source}\" passed to lookupTransform argument source_frame does not exist.";
            kind = ErrorKind.Lookup;
            return false;
        }

        if (!_ids.TryGetValue(target, out var targetId))
        {
            error = $"\"{target}\" passed to lookupTransform argument target_frame does not exist.";
            kind = ErrorKind.Lookup;
            return false;
        }

        if (sourceId == targetId)
        {
            return true;
        }

        if (timeNs == 0)
        {
            // Latest common time: the oldest of the newest samples on the connecting chain.
            if (!TryCommonTime(targetId, sourceId, out var common, out error, out kind))
            {
                return false;
            }

            stamp = common;
            timeNs = common;
        }

        return TryWalk(targetId, sourceId, timeNs, out result, out error, out kind);
    }

    private bool TryWalk(int targetId, int sourceId, long timeNs, out Transform3d result, out string? error, out ErrorKind kind)
    {
        result = Transform3d.Identity;
        error = null;
        kind = ErrorKind.None;

        // Walk up from the source, recording ancestor_T_source for every ancestor.
        Span<int> ancestorIds = stackalloc int[64];
        var ancestorTf = new Transform3d[64];
        var depth = 0;
        var frame = sourceId;
        var acc = Transform3d.Identity;
        ancestorIds[depth] = frame;
        ancestorTf[depth++] = acc;
        while (true)
        {
            var cache = CacheOf(frame);
            if (cache is null || !cache.HasData)
            {
                break;
            }

            if (!cache.TryGet(timeNs, out var sample, out error))
            {
                kind = ErrorKind.Extrapolation;
                return false;
            }

            acc = sample.Transform * acc;
            frame = sample.ParentId;
            if (depth >= MaxGraphDepth)
            {
                error = "The tf tree is invalid because it contains a loop.";
                kind = ErrorKind.Connectivity;
                return false;
            }

            if (depth == ancestorIds.Length)
            {
                var bigger = new int[ancestorIds.Length * 2];
                ancestorIds.CopyTo(bigger);
                ancestorIds = bigger;
                Array.Resize(ref ancestorTf, ancestorTf.Length * 2);
            }

            ancestorIds[depth] = frame;
            ancestorTf[depth++] = acc;
        }

        // Walk up from the target until we meet one of the source's ancestors.
        frame = targetId;
        var targetAcc = Transform3d.Identity;
        for (var steps = 0; steps <= MaxGraphDepth; steps++)
        {
            for (var i = 0; i < depth; i++)
            {
                if (ancestorIds[i] == frame)
                {
                    // target_T_source = inverse(common_T_target) * common_T_source
                    result = targetAcc.Inverse() * ancestorTf[i];
                    return true;
                }
            }

            var cache = CacheOf(frame);
            if (cache is null || !cache.HasData)
            {
                break;
            }

            if (!cache.TryGet(timeNs, out var sample, out error))
            {
                kind = ErrorKind.Extrapolation;
                return false;
            }

            targetAcc = sample.Transform * targetAcc;
            frame = sample.ParentId;
        }

        error = $"Could not find a connection between '{_names[targetId]}' and '{_names[sourceId]}' because they are not part of the same tree. Tf has two or more unconnected trees.";
        kind = ErrorKind.Connectivity;
        return false;
    }

    private bool TryCommonTime(int targetId, int sourceId, out long common, out string? error, out ErrorKind kind)
    {
        common = 0;
        error = null;
        kind = ErrorKind.None;

        // Chains (with each link's newest time) using the latest data.
        var sourceChain = Chain(sourceId);
        var targetChain = Chain(targetId);
        var meet = -1;
        var sourceIndex = -1;
        for (var ti = 0; ti < targetChain.Count && meet < 0; ti++)
        {
            for (var si = 0; si < sourceChain.Count; si++)
            {
                if (sourceChain[si].Frame == targetChain[ti].Frame)
                {
                    meet = ti;
                    sourceIndex = si;
                    break;
                }
            }
        }

        if (meet < 0)
        {
            error = $"Could not find a connection between '{_names[targetId]}' and '{_names[sourceId]}' because they are not part of the same tree. Tf has two or more unconnected trees.";
            kind = ErrorKind.Connectivity;
            return false;
        }

        var min = long.MaxValue;
        for (var i = 0; i < sourceIndex; i++)
        {
            if (sourceChain[i].Newest != 0)
            {
                min = Math.Min(min, sourceChain[i].Newest);
            }
        }

        for (var i = 0; i < meet; i++)
        {
            if (targetChain[i].Newest != 0)
            {
                min = Math.Min(min, targetChain[i].Newest);
            }
        }

        common = min == long.MaxValue ? 0 : min;
        return true;
    }

    private List<(int Frame, long Newest)> Chain(int frame)
    {
        var chain = new List<(int, long)>(8);
        for (var steps = 0; steps <= MaxGraphDepth; steps++)
        {
            var cache = CacheOf(frame);
            if (cache is null || !cache.HasData)
            {
                chain.Add((frame, 0));
                return chain;
            }

            chain.Add((frame, cache.TimeRange.Newest));
            frame = cache.LatestParent;
        }

        return chain;
    }

    private static TransformStamped ToStamped(string target, string source, Transform3d tf, long stampNs)
    {
        var msg = new TransformStamped { ChildFrameId = StripSlash(source), Transform = tf.ToMsg() };
        msg.Header.FrameId = StripSlash(target);
        new RosTime(stampNs).CopyTo(msg.Header.Stamp);
        return msg;
    }

    // ------------------------------------------------------------------ data helpers

    /// <summary>Transforms a stamped pose into <paramref name="targetFrame"/> at the pose's own stamp.</summary>
    public PoseStamped TransformPose(PoseStamped pose, string targetFrame, TimeSpan? timeout = null)
    {
        var tf = LookupFor(targetFrame, pose.Header.FrameId, RosTime.FromMsg(pose.Header.Stamp), timeout);
        var result = tf * Transform3d.FromMsg(pose.Pose);
        var msg = new PoseStamped { Pose = result.ToPoseMsg() };
        msg.Header.FrameId = StripSlash(targetFrame);
        msg.Header.Stamp = pose.Header.Stamp.Clone();
        return msg;
    }

    public PointStamped TransformPoint(PointStamped point, string targetFrame, TimeSpan? timeout = null)
    {
        var tf = LookupFor(targetFrame, point.Header.FrameId, RosTime.FromMsg(point.Header.Stamp), timeout);
        var msg = new PointStamped { Point = tf.TransformPoint(Vector3d.FromMsg(point.Point)).ToPointMsg() };
        msg.Header.FrameId = StripSlash(targetFrame);
        msg.Header.Stamp = point.Header.Stamp.Clone();
        return msg;
    }

    /// <summary>Rotates a stamped vector into <paramref name="targetFrame"/> (translation is ignored for vectors).</summary>
    public Vector3Stamped TransformVector(Vector3Stamped vector, string targetFrame, TimeSpan? timeout = null)
    {
        var tf = LookupFor(targetFrame, vector.Header.FrameId, RosTime.FromMsg(vector.Header.Stamp), timeout);
        var msg = new Vector3Stamped { Vector = tf.TransformVector(Vector3d.FromMsg(vector.Vector)).ToVector3Msg() };
        msg.Header.FrameId = StripSlash(targetFrame);
        msg.Header.Stamp = vector.Header.Stamp.Clone();
        return msg;
    }

    private Transform3d LookupFor(string target, string source, RosTime time, TimeSpan? timeout)
    {
        if (timeout is { } t && t > TimeSpan.Zero && !CanTransform(target, source, time))
        {
            WaitForTransformAsync(target, source, time, t).GetAwaiter().GetResult();
        }

        return LookupTransform3d(target, source, time);
    }

    // ------------------------------------------------------------------ introspection

    /// <summary>Every known frame name.</summary>
    public IReadOnlyList<string> FrameNames
    {
        get
        {
            lock (_registryGate)
            {
                return _names.Take(_count).Order(StringComparer.Ordinal).ToList();
            }
        }
    }

    public bool FrameExists(string frame) => _ids.ContainsKey(StripSlash(frame));

    /// <summary>The parent of <paramref name="frame"/> at <paramref name="time"/>, or null for roots and unknown frames.</summary>
    public string? GetParent(string frame, RosTime time = default)
    {
        if (!_ids.TryGetValue(StripSlash(frame), out var id) || CacheOf(id) is not { } cache || !cache.TryGet(time.Nanoseconds, out var sample, out _))
        {
            return null;
        }

        return _names[sample.ParentId];
    }

    /// <summary>The same text as <c>tf2::BufferCore::allFramesAsString</c>.</summary>
    public string AllFramesAsString()
    {
        var sb = new StringBuilder();
        foreach (var (name, cache) in Frames())
        {
            sb.Append("Frame ").Append(name).Append(" exists with parent ").Append(_names[cache.LatestParent]).Append(".\n");
        }

        return sb.ToString();
    }

    /// <summary>The YAML that <c>view_frames</c> / <c>tf2_monitor</c> consume (<c>allFramesAsYAML</c>).</summary>
    public string AllFramesAsYaml(RosTime? now = null)
    {
        var inv = CultureInfo.InvariantCulture;
        var current = (now ?? RosTime.Now()).Seconds;
        var sb = new StringBuilder();
        foreach (var (name, cache) in Frames())
        {
            var (oldest, newest) = cache.TimeRange;
            var elapsed = Math.Max(1e-3, (Environment.TickCount64 - cache.FirstInsertTicks) / 1000.0);
            var rate = cache.IsStatic ? 10000.0 : cache.InsertCount / elapsed;
            sb.Append(name).Append(": \n");
            sb.Append("  parent: '").Append(_names[cache.LatestParent]).Append("'\n");
            sb.Append("  broadcaster: '").Append(cache.Authority).Append("'\n");
            sb.Append("  rate: ").Append(rate.ToString("0.000", inv)).Append('\n');
            sb.Append("  most_recent_transform: ").Append((newest / 1e9).ToString("0.000000", inv)).Append('\n');
            sb.Append("  oldest_transform: ").Append((oldest / 1e9).ToString("0.000000", inv)).Append('\n');
            sb.Append("  buffer_length: ").Append(((newest - oldest) / 1e9).ToString("0.000", inv)).Append('\n');
            _ = current;
        }

        return sb.ToString();
    }

    /// <summary>The frame tree as a Mermaid flowchart.</summary>
    public string ToMermaid()
    {
        var sb = new StringBuilder("graph TD\n");
        foreach (var (name, cache) in Frames())
        {
            sb.Append("  ").Append(MermaidId(_names[cache.LatestParent])).Append("[\"").Append(_names[cache.LatestParent]).Append("\"] -->")
              .Append(cache.IsStatic ? "|static| " : " ")
              .Append(MermaidId(name)).Append("[\"").Append(name).Append("\"]\n");
        }

        return sb.ToString();
    }

    /// <summary>The frame tree in Graphviz DOT, like <c>view_frames</c>.</summary>
    public string ToDot()
    {
        var sb = new StringBuilder("digraph G {\n");
        foreach (var (name, cache) in Frames())
        {
            sb.Append("  \"").Append(_names[cache.LatestParent]).Append("\" -> \"").Append(name)
              .Append("\" [label=\"Broadcaster: ").Append(cache.Authority).Append(cache.IsStatic ? "\\nstatic" : string.Empty).Append("\"];\n");
        }

        return sb.Append("}\n").ToString();
    }

    private static string MermaidId(string name) => "f_" + new string(name.Select(c => char.IsLetterOrDigit(c) ? c : '_').ToArray());

    private IEnumerable<(string Name, TimeCache Cache)> Frames()
    {
        List<(string, TimeCache)> list;
        lock (_registryGate)
        {
            list = [];
            for (var i = 0; i < _count; i++)
            {
                if (_caches[i] is { HasData: true } c && c.LatestParent >= 0)
                {
                    list.Add((_names[i], c));
                }
            }
        }

        return list.OrderBy(x => x.Item1, StringComparer.Ordinal);
    }
}
