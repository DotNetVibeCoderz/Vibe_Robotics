using System.Security.Cryptography;
using System.Security.Cryptography.X509Certificates;
using ROS2.Net.Security;
using Shouldly;

namespace ROS2.Net.Integration.Tests.Security;

public sealed class SecurityTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "ros2net-keystore-" + Guid.NewGuid().ToString("N"));

    public void Dispose()
    {
        if (Directory.Exists(_dir))
        {
            Directory.Delete(_dir, recursive: true);
        }
    }

    [Fact]
    public void Keystore_has_the_sros2_layout()
    {
        var ks = SrosKeystore.Create(_dir);
        foreach (var f in new[]
        {
            "public/ca.cert.pem", "public/identity_ca.cert.pem", "public/permissions_ca.cert.pem",
            "private/ca.key.pem", "private/identity_ca.key.pem", "private/permissions_ca.key.pem",
            "enclaves/governance.xml", "enclaves/governance.p7s",
        })
        {
            File.Exists(Path.Combine(_dir, f)).ShouldBeTrue(f);
        }

        using var ca = ks.LoadCa();
        ca.Subject.ShouldBe("CN=sros2testCA");
        ca.GetECDsaPublicKey()!.KeySize.ShouldBe(256);
        SrosKeystore.IsValid(_dir).ShouldBeTrue();

        var governance = SmimeSigner.Verify(File.ReadAllText(ks.GovernancePath), ks.LoadCaCertificate());
        governance.ShouldContain("<rtps_protection_kind>SIGN</rtps_protection_kind>");
    }

    [Fact]
    public void Enclave_is_signed_by_the_keystore_ca()
    {
        var ks = SrosKeystore.Create(_dir);
        var dir = ks.CreateEnclave("/talker_listener/talker");

        Directory.Exists(Path.Combine(_dir, "enclaves", "talker_listener", "talker")).ShouldBeTrue();
        using var cert = X509Certificate2.CreateFromPemFile(Path.Combine(dir, "cert.pem"), Path.Combine(dir, "key.pem"));
        cert.Subject.ShouldBe("CN=/talker_listener/talker");
        cert.HasPrivateKey.ShouldBeTrue();
        SmimeSigner.ChainsTo(cert, ks.LoadCaCertificate()).ShouldBeTrue();

        var verification = ks.VerifyEnclave("/talker_listener/talker");
        verification.Problems.ShouldBeEmpty();
        ks.ListEnclaves().ShouldBe(new[] { "/talker_listener/talker" });

        var permissions = SmimeSigner.Verify(File.ReadAllText(Path.Combine(dir, "permissions.p7s")), ks.LoadCaCertificate());
        permissions.ShouldContain("<subject_name>CN=/talker_listener/talker</subject_name>");
        permissions.ShouldContain("<topic>*</topic>");
    }

    [Fact]
    public void Tampered_or_foreign_signatures_are_rejected()
    {
        var ks = SrosKeystore.Create(_dir);
        var signed = File.ReadAllText(ks.GovernancePath);
        Should.Throw<CryptographicException>(() =>
            SmimeSigner.Verify(signed.Replace("ENCRYPT", "NONE"), ks.LoadCaCertificate()));

        var other = SrosKeystore.Create(_dir + "-other");
        try
        {
            Should.Throw<CryptographicException>(() => SmimeSigner.Verify(signed, other.LoadCaCertificate()));
        }
        finally
        {
            Directory.Delete(other.Root, recursive: true);
        }
    }

    [Fact]
    public void Policies_become_mangled_dds_permissions()
    {
        const string policyXml = """
            <policy version="0.2.0">
              <enclaves>
                <enclave path="/demo/talker">
                  <profiles>
                    <profile ns="/" node="talker">
                      <topics publish="ALLOW"><topic>chatter</topic></topics>
                      <topics subscribe="DENY"><topic>/secret</topic></topics>
                      <services reply="ALLOW"><service>~/reset</service></services>
                      <actions call="ALLOW"><action>fibonacci</action></actions>
                    </profile>
                  </profiles>
                </enclave>
              </enclaves>
            </policy>
            """;
        var policy = SecurityPolicy.Parse(policyXml);
        var ks = SrosKeystore.Create(_dir);
        ks.CreateEnclaves(policy);

        var xml = File.ReadAllText(Path.Combine(ks.EnclaveDirectory("/demo/talker"), "permissions.xml"));
        xml.ShouldContain("<topic>rt/chatter</topic>");
        xml.ShouldContain("<topic>rt/secret</topic>");
        xml.ShouldContain("<deny_rule>");
        xml.ShouldContain("<topic>rq/talker/resetRequest</topic>");
        xml.ShouldContain("<topic>rr/talker/resetReply</topic>");
        xml.ShouldContain("<topic>rq/fibonacci/_action/send_goalRequest</topic>");
        xml.ShouldContain("<topic>rt/rosout</topic>");
        ks.VerifyEnclave("/demo/talker").IsValid.ShouldBeTrue();
    }
}
