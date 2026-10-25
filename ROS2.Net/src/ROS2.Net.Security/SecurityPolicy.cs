using System.Xml.Linq;

namespace ROS2.Net.Security;

/// <summary>Allow or deny.</summary>
public enum AccessMode
{
    Allow,
    Deny,
}

/// <summary>Access to one topic. Null means "not mentioned" for that direction.</summary>
public sealed record TopicRule(string Name, AccessMode? Publish = null, AccessMode? Subscribe = null);

/// <summary>Access to one service: <c>Reply</c> lets the enclave serve it, <c>Request</c> lets it call it.</summary>
public sealed record ServiceRule(string Name, AccessMode? Reply = null, AccessMode? Request = null);

/// <summary>Access to one action: <c>Execute</c> lets the enclave serve it, <c>Call</c> lets it send goals.</summary>
public sealed record ActionRule(string Name, AccessMode? Execute = null, AccessMode? Call = null);

/// <summary>The permissions of one enclave. Names are fully qualified ROS names (e.g. <c>/chatter</c>).</summary>
public sealed class EnclavePolicy
{
    public EnclavePolicy(string path) => Path = path;

    /// <summary>Enclave path, e.g. <c>/talker_listener/talker</c>.</summary>
    public string Path { get; }

    public List<TopicRule> Topics { get; } = [];

    public List<ServiceRule> Services { get; } = [];

    public List<ActionRule> Actions { get; } = [];

    /// <summary>Add the topics every ROS 2 node uses (<c>/rosout</c>, <c>/parameter_events</c>, <c>ros_discovery_info</c>, parameter services).</summary>
    public bool IncludeDefaults { get; set; } = true;

    public EnclavePolicy AllowPublish(params string[] topics)
    {
        Topics.AddRange(topics.Select(t => new TopicRule(t, Publish: AccessMode.Allow)));
        return this;
    }

    public EnclavePolicy AllowSubscribe(params string[] topics)
    {
        Topics.AddRange(topics.Select(t => new TopicRule(t, Subscribe: AccessMode.Allow)));
        return this;
    }

    public EnclavePolicy AllowServe(params string[] services)
    {
        Services.AddRange(services.Select(s => new ServiceRule(s, Reply: AccessMode.Allow)));
        return this;
    }

    public EnclavePolicy AllowCall(params string[] services)
    {
        Services.AddRange(services.Select(s => new ServiceRule(s, Request: AccessMode.Allow)));
        return this;
    }
}

/// <summary>
/// A set of enclave policies. Reads the sros2 policy format
/// (<c>&lt;policy version="0.2.0"&gt;&lt;enclaves&gt;&lt;enclave path=".."&gt;&lt;profiles&gt;&lt;profile ns=".." node=".."&gt;...</c>).
/// </summary>
public sealed class SecurityPolicy
{
    public List<EnclavePolicy> Enclaves { get; } = [];

    public EnclavePolicy? Find(string enclavePath) =>
        Enclaves.FirstOrDefault(e => e.Path == SrosKeystore.NormalizeEnclave(enclavePath));

    public static SecurityPolicy Load(string path) => Parse(File.ReadAllText(path));

    public static SecurityPolicy Parse(string xml)
    {
        var doc = XDocument.Parse(xml);
        var policy = new SecurityPolicy();
        foreach (var enclave in doc.Descendants("enclave"))
        {
            var ep = new EnclavePolicy(SrosKeystore.NormalizeEnclave((string?)enclave.Attribute("path") ?? "/"));
            foreach (var profile in enclave.Descendants("profile"))
            {
                var ns = (string?)profile.Attribute("ns") ?? "/";
                var node = (string?)profile.Attribute("node") ?? string.Empty;
                foreach (var group in profile.Elements("topics"))
                {
                    var pub = Mode(group.Attribute("publish"));
                    var sub = Mode(group.Attribute("subscribe"));
                    foreach (var t in group.Elements("topic"))
                    {
                        ep.Topics.Add(new TopicRule(Expand(t.Value.Trim(), ns, node), pub, sub));
                    }
                }

                foreach (var group in profile.Elements("services"))
                {
                    var reply = Mode(group.Attribute("reply"));
                    var request = Mode(group.Attribute("request"));
                    foreach (var s in group.Elements("service"))
                    {
                        ep.Services.Add(new ServiceRule(Expand(s.Value.Trim(), ns, node), reply, request));
                    }
                }

                foreach (var group in profile.Elements("actions"))
                {
                    var execute = Mode(group.Attribute("execute"));
                    var call = Mode(group.Attribute("call"));
                    foreach (var a in group.Elements("action"))
                    {
                        ep.Actions.Add(new ActionRule(Expand(a.Value.Trim(), ns, node), execute, call));
                    }
                }
            }

            policy.Enclaves.Add(ep);
        }

        return policy;
    }

    private static AccessMode? Mode(XAttribute? a) => (string?)a switch
    {
        null => null,
        var v when v.Equals("ALLOW", StringComparison.OrdinalIgnoreCase) => AccessMode.Allow,
        _ => AccessMode.Deny,
    };

    /// <summary>Expands a policy name relative to its profile: absolute stays, <c>~x</c> is private, otherwise ns-relative.</summary>
    internal static string Expand(string name, string ns, string node)
    {
        var n = ns.TrimEnd('/');
        if (name.StartsWith('/'))
        {
            return name;
        }

        if (name.StartsWith('~'))
        {
            return $"{n}/{node}{name[1..]}".Replace("//", "/");
        }

        return $"{n}/{name}";
    }
}
