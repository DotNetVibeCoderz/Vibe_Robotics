using System.Globalization;
using System.Xml.Linq;

namespace ROS2.Net.Security;

/// <summary>Builds the OMG DDS-Security governance and permissions XML documents that SROS2 signs.</summary>
public static class DdsSecurityDocuments
{
    private static readonly XNamespace s_xsi = "http://www.w3.org/2001/XMLSchema-instance";

    /// <summary>The governance document sros2 generates: every domain protected, discovery/liveliness encrypted, RTPS signed, all topics access-controlled and encrypted.</summary>
    public static string Governance(int minDomain = 0, int maxDomain = 230) =>
        Serialize(new XElement("dds",
            new XAttribute(XNamespace.Xmlns + "xsi", s_xsi),
            new XAttribute(s_xsi + "noNamespaceSchemaLocation", "http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_governance.xsd"),
            new XElement("domain_access_rules",
                new XElement("domain_rule",
                    Domains(minDomain, maxDomain),
                    new XElement("allow_unauthenticated_participants", "false"),
                    new XElement("enable_join_access_control", "true"),
                    new XElement("discovery_protection_kind", "ENCRYPT"),
                    new XElement("liveliness_protection_kind", "ENCRYPT"),
                    new XElement("rtps_protection_kind", "SIGN"),
                    new XElement("topic_access_rules",
                        new XElement("topic_rule",
                            new XElement("topic_expression", "*"),
                            new XElement("enable_discovery_protection", "true"),
                            new XElement("enable_liveliness_protection", "true"),
                            new XElement("enable_read_access_control", "true"),
                            new XElement("enable_write_access_control", "true"),
                            new XElement("metadata_protection_kind", "ENCRYPT"),
                            new XElement("data_protection_kind", "ENCRYPT")))))));

    /// <summary>
    /// The permissions document of one enclave. With no policy every topic is allowed (the sros2 default);
    /// with a policy, ROS names are mangled to DDS topics (<c>rt/</c>, <c>rq/..Request</c>, <c>rr/..Reply</c>) and everything else is denied.
    /// </summary>
    public static string Permissions(string enclave, string subjectName, DateTimeOffset notBefore, DateTimeOffset notAfter, EnclavePolicy? policy, int minDomain = 0, int maxDomain = 230)
    {
        var rules = new List<XElement>();
        if (policy is null)
        {
            rules.Add(new XElement("allow_rule",
                Domains(minDomain, maxDomain),
                new XElement("publish", Topics(["*"])),
                new XElement("subscribe", Topics(["*"]))));
        }
        else
        {
            var (allowPub, allowSub, denyPub, denySub) = Mangle(policy);
            if (denyPub.Count + denySub.Count > 0)
            {
                rules.Add(new XElement("deny_rule", Domains(minDomain, maxDomain),
                    denyPub.Count > 0 ? new XElement("publish", Topics(denyPub)) : null,
                    denySub.Count > 0 ? new XElement("subscribe", Topics(denySub)) : null));
            }

            rules.Add(new XElement("allow_rule", Domains(minDomain, maxDomain),
                allowPub.Count > 0 ? new XElement("publish", Topics(allowPub)) : null,
                allowSub.Count > 0 ? new XElement("subscribe", Topics(allowSub)) : null));
        }

        return Serialize(new XElement("dds",
            new XAttribute(XNamespace.Xmlns + "xsi", s_xsi),
            new XAttribute(s_xsi + "noNamespaceSchemaLocation", "http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_permissions.xsd"),
            new XElement("permissions",
                new XElement("grant",
                    new XAttribute("name", enclave),
                    new XElement("subject_name", subjectName),
                    new XElement("validity",
                        new XElement("not_before", Stamp(notBefore)),
                        new XElement("not_after", Stamp(notAfter))),
                    rules,
                    new XElement("default", "DENY")))));
    }

    /// <summary>Maps a ROS policy onto DDS topic names.</summary>
    internal static (List<string> AllowPub, List<string> AllowSub, List<string> DenyPub, List<string> DenySub) Mangle(EnclavePolicy policy)
    {
        var allowPub = new SortedSet<string>(StringComparer.Ordinal);
        var allowSub = new SortedSet<string>(StringComparer.Ordinal);
        var denyPub = new SortedSet<string>(StringComparer.Ordinal);
        var denySub = new SortedSet<string>(StringComparer.Ordinal);

        void Add(AccessMode? mode, SortedSet<string> allow, SortedSet<string> deny, string topic)
        {
            if (mode == AccessMode.Allow)
            {
                allow.Add(topic);
            }
            else if (mode == AccessMode.Deny)
            {
                deny.Add(topic);
            }
        }

        static string Rel(string name) => name.TrimStart('/');

        foreach (var t in policy.Topics)
        {
            Add(t.Publish, allowPub, denyPub, "rt/" + Rel(t.Name));
            Add(t.Subscribe, allowSub, denySub, "rt/" + Rel(t.Name));
        }

        foreach (var s in policy.Services)
        {
            // Server: reads requests, writes replies. Client: the opposite.
            Add(s.Reply, allowSub, denySub, $"rq/{Rel(s.Name)}Request");
            Add(s.Reply, allowPub, denyPub, $"rr/{Rel(s.Name)}Reply");
            Add(s.Request, allowPub, denyPub, $"rq/{Rel(s.Name)}Request");
            Add(s.Request, allowSub, denySub, $"rr/{Rel(s.Name)}Reply");
        }

        foreach (var a in policy.Actions)
        {
            var n = Rel(a.Name);
            foreach (var svc in new[] { "send_goal", "cancel_goal", "get_result" })
            {
                Add(a.Execute, allowSub, denySub, $"rq/{n}/_action/{svc}Request");
                Add(a.Execute, allowPub, denyPub, $"rr/{n}/_action/{svc}Reply");
                Add(a.Call, allowPub, denyPub, $"rq/{n}/_action/{svc}Request");
                Add(a.Call, allowSub, denySub, $"rr/{n}/_action/{svc}Reply");
            }

            foreach (var topic in new[] { "feedback", "status" })
            {
                Add(a.Execute, allowPub, denyPub, $"rt/{n}/_action/{topic}");
                Add(a.Call, allowSub, denySub, $"rt/{n}/_action/{topic}");
            }
        }

        if (policy.IncludeDefaults)
        {
            allowPub.UnionWith(["ros_discovery_info", "rt/rosout", "rt/parameter_events", "rr/*/get_parameters*Reply", "rr/*/set_parameters*Reply", "rr/*/list_parameters*Reply", "rr/*/describe_parameters*Reply", "rr/*/get_parameter_types*Reply"]);
            allowSub.UnionWith(["ros_discovery_info", "rt/parameter_events", "rt/clock", "rq/*/get_parameters*Request", "rq/*/set_parameters*Request", "rq/*/list_parameters*Request", "rq/*/describe_parameters*Request", "rq/*/get_parameter_types*Request"]);
        }

        return (allowPub.ToList(), allowSub.ToList(), denyPub.ToList(), denySub.ToList());
    }

    private static XElement Domains(int min, int max) =>
        new("domains", new XElement("id_range", new XElement("min", min), new XElement("max", max)));

    private static XElement Topics(IEnumerable<string> names) => new("topics", names.Select(n => new XElement("topic", n)));

    private static string Stamp(DateTimeOffset t) => t.UtcDateTime.ToString("yyyy-MM-ddTHH:mm:ss", CultureInfo.InvariantCulture);

    private static string Serialize(XElement root) =>
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" + root.ToString() + "\n";
}
