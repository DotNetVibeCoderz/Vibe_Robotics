namespace ROS2.Net.Tf2;

/// <summary>One stored transform: the pose of a child frame in <see cref="ParentId"/> at a time.</summary>
internal readonly record struct TfSample(long TimeNs, int ParentId, Transform3d Transform);

/// <summary>
/// The time-ordered history of one child frame, like <c>tf2::TimeCache</c>. Samples are kept in a
/// sorted list (appends are O(1) in the common in-order case) and pruned to the cache duration.
/// </summary>
internal sealed class TimeCache
{
    private readonly object _gate = new();
    private readonly List<TfSample> _samples = new(16);
    private readonly long _maxAgeNs;
    private TfSample _static;

    public TimeCache(string frameId, int frameIndex, long maxAgeNs, bool isStatic)
    {
        FrameId = frameId;
        FrameIndex = frameIndex;
        _maxAgeNs = maxAgeNs;
        IsStatic = isStatic;
    }

    public string FrameId { get; }

    public int FrameIndex { get; }

    /// <summary>Static frames hold exactly one transform valid at every time.</summary>
    public bool IsStatic { get; private set; }

    public string Authority { get; set; } = "unknown";

    public long InsertCount { get; private set; }

    public long FirstInsertTicks { get; private set; } = Environment.TickCount64;

    public bool Insert(TfSample sample, bool isStatic)
    {
        lock (_gate)
        {
            InsertCount++;
            if (isStatic)
            {
                IsStatic = true;
                _static = sample with { TimeNs = 0 };
                _samples.Clear();
                return true;
            }

            if (IsStatic)
            {
                // A dynamic sample for a frame published as static: dynamic data wins, like tf2's per-child caches.
                IsStatic = false;
            }

            var n = _samples.Count;
            if (n > 0)
            {
                var newest = _samples[n - 1].TimeNs;
                if (sample.TimeNs < newest - _maxAgeNs)
                {
                    return false; // Older than the cache window: tf2 drops these with a warning.
                }
            }

            if (n == 0 || sample.TimeNs > _samples[n - 1].TimeNs)
            {
                _samples.Add(sample);
            }
            else
            {
                var i = LowerBound(sample.TimeNs);
                if (i < n && _samples[i].TimeNs == sample.TimeNs)
                {
                    _samples[i] = sample; // Same stamp: replace.
                }
                else
                {
                    _samples.Insert(i, sample);
                }
            }

            Prune();
            return true;
        }
    }

    private void Prune()
    {
        var newest = _samples[^1].TimeNs;
        var cut = 0;
        while (cut < _samples.Count - 1 && _samples[cut].TimeNs < newest - _maxAgeNs)
        {
            cut++;
        }

        if (cut > 0)
        {
            _samples.RemoveRange(0, cut);
        }
    }

    private int LowerBound(long t)
    {
        int lo = 0, hi = _samples.Count;
        while (lo < hi)
        {
            var mid = (lo + hi) >>> 1;
            if (_samples[mid].TimeNs < t)
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid;
            }
        }

        return lo;
    }

    /// <summary>Range of cached times, (0, 0) for static frames.</summary>
    public (long Oldest, long Newest) TimeRange
    {
        get
        {
            lock (_gate)
            {
                return IsStatic || _samples.Count == 0 ? (0, 0) : (_samples[0].TimeNs, _samples[^1].TimeNs);
            }
        }
    }

    public bool HasData
    {
        get
        {
            lock (_gate)
            {
                return IsStatic || _samples.Count > 0;
            }
        }
    }

    /// <summary>
    /// The sample at <paramref name="timeNs"/> (0 = latest), interpolated between neighbours. Returns
    /// an error message instead of throwing, so lookups can probe without exceptions.
    /// </summary>
    public bool TryGet(long timeNs, out TfSample sample, out string? error)
    {
        lock (_gate)
        {
            error = null;
            if (IsStatic)
            {
                sample = _static;
                return true;
            }

            var n = _samples.Count;
            if (n == 0)
            {
                sample = default;
                error = $"Frame '{FrameId}' has no data.";
                return false;
            }

            if (timeNs == 0)
            {
                sample = _samples[n - 1];
                return true;
            }

            if (n == 1)
            {
                if (_samples[0].TimeNs == timeNs)
                {
                    sample = _samples[0];
                    return true;
                }

                sample = default;
                error = $"Lookup would require extrapolation: frame '{FrameId}' has only one sample at {new RosTime(_samples[0].TimeNs)}, requested {new RosTime(timeNs)}.";
                return false;
            }

            var oldest = _samples[0].TimeNs;
            var newest = _samples[n - 1].TimeNs;
            if (timeNs < oldest || timeNs > newest)
            {
                sample = default;
                var dir = timeNs < oldest ? "into the past" : "into the future";
                error = $"Lookup would require extrapolation {dir}. Requested time {new RosTime(timeNs)} but the {(timeNs < oldest ? "earliest" : "latest")} data for frame '{FrameId}' is at {new RosTime(timeNs < oldest ? oldest : newest)}.";
                return false;
            }

            var i = LowerBound(timeNs);
            var after = _samples[i];
            if (after.TimeNs == timeNs)
            {
                sample = after;
                return true;
            }

            var before = _samples[i - 1];
            if (before.ParentId != after.ParentId)
            {
                // The parent changed between samples: interpolation is meaningless, use the earlier one.
                sample = before;
                return true;
            }

            var ratio = (double)(timeNs - before.TimeNs) / (after.TimeNs - before.TimeNs);
            sample = new TfSample(timeNs, before.ParentId, Transform3d.Interpolate(before.Transform, after.Transform, ratio));
            return true;
        }
    }

    public int Count
    {
        get
        {
            lock (_gate)
            {
                return IsStatic ? 1 : _samples.Count;
            }
        }
    }

    public int LatestParent
    {
        get
        {
            lock (_gate)
            {
                return IsStatic ? _static.ParentId : _samples.Count > 0 ? _samples[^1].ParentId : -1;
            }
        }
    }
}
