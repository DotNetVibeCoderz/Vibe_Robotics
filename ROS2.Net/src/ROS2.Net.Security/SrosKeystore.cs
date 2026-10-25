using System.Security.Cryptography;
using System.Security.Cryptography.X509Certificates;

namespace ROS2.Net.Security;

/// <summary>Options for creating keystores and enclaves.</summary>
public sealed class KeystoreOptions
{
    /// <summary>Subject of the keystore CA (sros2 uses <c>CN=sros2testCA</c>).</summary>
    public string CaSubject { get; init; } = "CN=sros2testCA";

    public TimeSpan CaValidity { get; init; } = TimeSpan.FromDays(3650);

    public TimeSpan EnclaveValidity { get; init; } = TimeSpan.FromDays(3650);

    public int MinDomainId { get; init; }

    public int MaxDomainId { get; init; } = 230;
}

/// <summary>
/// An SROS2 keystore in the layout <c>ros2 security create_keystore</c> produces:
/// <code>
/// keystore/
///   enclaves/governance.xml, governance.p7s, &lt;enclave&gt;/{cert.pem, key.pem, identity_ca.cert.pem,
///            permissions_ca.cert.pem, governance.p7s, permissions.xml, permissions.p7s}
///   private/ca.key.pem, identity_ca.key.pem, permissions_ca.key.pem
///   public/ca.cert.pem, identity_ca.cert.pem, permissions_ca.cert.pem
/// </code>
/// sros2 links the identity/permissions CA files to the single CA; this class writes copies, which
/// works on every platform. Point a DDS deployment at it with <c>ROS_SECURITY_KEYSTORE</c>,
/// <c>ROS_SECURITY_ENABLE=true</c> and <c>ROS_SECURITY_STRATEGY=Enforce</c>.
/// </summary>
public sealed class SrosKeystore
{
    private SrosKeystore(string root) => Root = Path.GetFullPath(root);

    public string Root { get; }

    public string EnclavesDirectory => Path.Combine(Root, "enclaves");

    public string CaCertificatePath => Path.Combine(Root, "public", "ca.cert.pem");

    public string CaKeyPath => Path.Combine(Root, "private", "ca.key.pem");

    public string GovernancePath => Path.Combine(EnclavesDirectory, "governance.p7s");

    /// <summary>Opens an existing keystore (throws if it is not one).</summary>
    public static SrosKeystore Open(string root)
    {
        var ks = new SrosKeystore(root);
        if (!IsValid(root))
        {
            throw new InvalidOperationException($"'{ks.Root}' is not an SROS2 keystore.");
        }

        return ks;
    }

    /// <summary>True when <paramref name="root"/> has the keystore files.</summary>
    public static bool IsValid(string root) =>
        File.Exists(Path.Combine(root, "public", "ca.cert.pem")) &&
        File.Exists(Path.Combine(root, "private", "ca.key.pem")) &&
        File.Exists(Path.Combine(root, "enclaves", "governance.p7s"));

    /// <summary>Creates a keystore: a self-signed ECDSA P-256 CA, and signed governance. Existing keystores are left untouched.</summary>
    public static SrosKeystore Create(string root, KeystoreOptions? options = null)
    {
        options ??= new KeystoreOptions();
        var ks = new SrosKeystore(root);
        if (IsValid(root))
        {
            return ks;
        }

        Directory.CreateDirectory(Path.Combine(ks.Root, "public"));
        Directory.CreateDirectory(Path.Combine(ks.Root, "private"));
        Directory.CreateDirectory(ks.EnclavesDirectory);

        using var key = ECDsa.Create(ECCurve.NamedCurves.nistP256);
        var request = new CertificateRequest(options.CaSubject, key, HashAlgorithmName.SHA256);
        request.CertificateExtensions.Add(new X509BasicConstraintsExtension(true, false, 0, true));
        request.CertificateExtensions.Add(new X509KeyUsageExtension(X509KeyUsageFlags.KeyCertSign | X509KeyUsageFlags.CrlSign | X509KeyUsageFlags.DigitalSignature, true));
        request.CertificateExtensions.Add(new X509SubjectKeyIdentifierExtension(request.PublicKey, false));
        var now = DateTimeOffset.UtcNow.AddMinutes(-5);
        using var ca = request.CreateSelfSigned(now, now + options.CaValidity);

        var certPem = ca.ExportCertificatePem() + "\n";
        var keyPem = key.ExportPkcs8PrivateKeyPem() + "\n";
        foreach (var name in new[] { "ca", "identity_ca", "permissions_ca" })
        {
            File.WriteAllText(Path.Combine(ks.Root, "public", name + ".cert.pem"), certPem);
            File.WriteAllText(Path.Combine(ks.Root, "private", name + ".key.pem"), keyPem);
        }

        var governance = DdsSecurityDocuments.Governance(options.MinDomainId, options.MaxDomainId);
        File.WriteAllText(Path.Combine(ks.EnclavesDirectory, "governance.xml"), governance);
        File.WriteAllText(ks.GovernancePath, SmimeSigner.Sign(governance, ca));
        return ks;
    }

    /// <summary>Loads the CA certificate together with its private key (for signing).</summary>
    public X509Certificate2 LoadCa() => X509Certificate2.CreateFromPemFile(CaCertificatePath, CaKeyPath);

    /// <summary>Loads the CA certificate without its key (for verification).</summary>
    public X509Certificate2 LoadCaCertificate() => X509Certificate2.CreateFromPem(File.ReadAllText(CaCertificatePath));

    /// <summary>Directory of an enclave, e.g. <c>/talker_listener/talker</c> → <c>enclaves/talker_listener/talker</c>.</summary>
    public string EnclaveDirectory(string enclave) =>
        Path.Combine([EnclavesDirectory, .. NormalizeEnclave(enclave).Split('/', StringSplitOptions.RemoveEmptyEntries)]);

    /// <summary>Lists the enclaves in the keystore.</summary>
    public IReadOnlyList<string> ListEnclaves() =>
        Directory.EnumerateFiles(EnclavesDirectory, "cert.pem", SearchOption.AllDirectories)
            .Select(f => "/" + Path.GetRelativePath(EnclavesDirectory, Path.GetDirectoryName(f)!).Replace('\\', '/'))
            .Order(StringComparer.Ordinal)
            .ToList();

    /// <summary>
    /// Creates (or regenerates) an enclave: an identity certificate signed by the CA, a copy of the
    /// signed governance, and permissions generated from <paramref name="policy"/> (allow-all when null).
    /// </summary>
    public string CreateEnclave(string enclave, EnclavePolicy? policy = null, KeystoreOptions? options = null)
    {
        options ??= new KeystoreOptions();
        enclave = NormalizeEnclave(enclave);
        var dir = EnclaveDirectory(enclave);
        Directory.CreateDirectory(dir);

        using var ca = LoadCa();
        using var key = ECDsa.Create(ECCurve.NamedCurves.nistP256);
        var subject = "CN=" + enclave;
        var request = new CertificateRequest(subject, key, HashAlgorithmName.SHA256);
        request.CertificateExtensions.Add(new X509BasicConstraintsExtension(false, false, 0, true));
        request.CertificateExtensions.Add(new X509KeyUsageExtension(X509KeyUsageFlags.DigitalSignature | X509KeyUsageFlags.KeyAgreement, true));
        var now = DateTimeOffset.UtcNow.AddMinutes(-5);
        var notAfter = now + options.EnclaveValidity;
        if (notAfter > ca.NotAfter)
        {
            notAfter = ca.NotAfter;
        }

        var serial = new byte[16];
        RandomNumberGenerator.Fill(serial);
        serial[0] &= 0x7f;
        using var cert = request.Create(ca, now, notAfter, serial);

        File.WriteAllText(Path.Combine(dir, "cert.pem"), cert.ExportCertificatePem() + "\n");
        File.WriteAllText(Path.Combine(dir, "key.pem"), key.ExportPkcs8PrivateKeyPem() + "\n");
        File.Copy(Path.Combine(Root, "public", "identity_ca.cert.pem"), Path.Combine(dir, "identity_ca.cert.pem"), true);
        File.Copy(Path.Combine(Root, "public", "permissions_ca.cert.pem"), Path.Combine(dir, "permissions_ca.cert.pem"), true);
        File.Copy(GovernancePath, Path.Combine(dir, "governance.p7s"), true);

        WritePermissions(enclave, policy, ca, cert.Subject, now, notAfter, options);
        return dir;
    }

    /// <summary>Regenerates <c>permissions.xml</c>/<c>.p7s</c> of an existing enclave from a policy.</summary>
    public void UpdatePermissions(string enclave, EnclavePolicy? policy, KeystoreOptions? options = null)
    {
        options ??= new KeystoreOptions();
        var dir = EnclaveDirectory(enclave);
        using var cert = X509Certificate2.CreateFromPem(File.ReadAllText(Path.Combine(dir, "cert.pem")));
        using var ca = LoadCa();
        WritePermissions(NormalizeEnclave(enclave), policy, ca, cert.Subject, cert.NotBefore, cert.NotAfter, options);
    }

    /// <summary>Creates every enclave of a policy file.</summary>
    public IReadOnlyList<string> CreateEnclaves(SecurityPolicy policy, KeystoreOptions? options = null) =>
        policy.Enclaves.Select(e => CreateEnclave(e.Path, e, options)).ToList();

    private void WritePermissions(string enclave, EnclavePolicy? policy, X509Certificate2 ca, string subject, DateTimeOffset notBefore, DateTimeOffset notAfter, KeystoreOptions options)
    {
        var dir = EnclaveDirectory(enclave);
        var xml = DdsSecurityDocuments.Permissions(enclave, subject, notBefore, notAfter, policy, options.MinDomainId, options.MaxDomainId);
        File.WriteAllText(Path.Combine(dir, "permissions.xml"), xml);
        File.WriteAllText(Path.Combine(dir, "permissions.p7s"), SmimeSigner.Sign(xml, ca));
    }

    /// <summary>Checks an enclave: files present, identity certificate issued by the CA, governance and permissions signatures valid.</summary>
    public KeystoreVerification VerifyEnclave(string enclave)
    {
        var problems = new List<string>();
        var dir = EnclaveDirectory(enclave);
        foreach (var f in new[] { "cert.pem", "key.pem", "identity_ca.cert.pem", "permissions_ca.cert.pem", "governance.p7s", "permissions.xml", "permissions.p7s" })
        {
            if (!File.Exists(Path.Combine(dir, f)))
            {
                problems.Add($"missing {f}");
            }
        }

        if (problems.Count > 0)
        {
            return new KeystoreVerification(false, problems);
        }

        using var ca = LoadCaCertificate();
        try
        {
            using var cert = X509Certificate2.CreateFromPemFile(Path.Combine(dir, "cert.pem"), Path.Combine(dir, "key.pem"));
            if (!SmimeSigner.ChainsTo(cert, ca))
            {
                problems.Add("cert.pem is not issued by the keystore CA");
            }

            if (cert.Subject != "CN=" + NormalizeEnclave(enclave))
            {
                problems.Add($"cert.pem subject '{cert.Subject}' does not name the enclave");
            }
        }
        catch (CryptographicException ex)
        {
            problems.Add("cert.pem/key.pem: " + ex.Message);
        }

        foreach (var signed in new[] { "governance.p7s", "permissions.p7s" })
        {
            try
            {
                SmimeSigner.Verify(File.ReadAllText(Path.Combine(dir, signed)), ca);
            }
            catch (CryptographicException ex)
            {
                problems.Add($"{signed}: {ex.Message}");
            }
        }

        return new KeystoreVerification(problems.Count == 0, problems);
    }

    /// <summary>Normalizes an enclave path to <c>/a/b</c>.</summary>
    public static string NormalizeEnclave(string enclave)
    {
        var trimmed = enclave.Replace('\\', '/').Trim('/');
        return "/" + trimmed;
    }
}

/// <summary>Result of <see cref="SrosKeystore.VerifyEnclave"/>.</summary>
public sealed record KeystoreVerification(bool IsValid, IReadOnlyList<string> Problems);
