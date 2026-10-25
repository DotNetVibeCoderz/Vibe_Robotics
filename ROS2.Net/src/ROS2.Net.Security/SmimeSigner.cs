using System.Security.Cryptography;
using System.Security.Cryptography.Pkcs;
using System.Security.Cryptography.X509Certificates;
using System.Text;

namespace ROS2.Net.Security;

/// <summary>
/// S/MIME detached signatures (PKCS#7, SHA-256), the format SROS2 uses for <c>governance.p7s</c> and
/// <c>permissions.p7s</c> and that Fast DDS and Cyclone DDS accept.
/// </summary>
public static class SmimeSigner
{
    private const string Boundary = "----ROS2NET-SMIME-BOUNDARY";

    /// <summary>Signs <paramref name="document"/> with <paramref name="signer"/> (which must carry its private key).</summary>
    public static string Sign(string document, X509Certificate2 signer)
    {
        var content = Canonical(document);
        var cms = new SignedCms(new ContentInfo(content), detached: true);
        cms.ComputeSignature(new CmsSigner(SubjectIdentifierType.IssuerAndSerialNumber, signer)
        {
            DigestAlgorithm = new Oid("2.16.840.1.101.3.4.2.1"), // SHA-256
            IncludeOption = X509IncludeOption.EndCertOnly,
        });
        var signature = Convert.ToBase64String(cms.Encode(), Base64FormattingOptions.InsertLineBreaks).Replace("\r\n", "\n").Replace("\n", "\r\n");

        var sb = new StringBuilder();
        sb.Append("MIME-Version: 1.0\r\n");
        sb.Append($"Content-Type: multipart/signed; protocol=\"application/x-pkcs7-signature\"; micalg=\"sha-256\"; boundary=\"{Boundary}\"\r\n\r\n");
        sb.Append("This is an S/MIME signed message\r\n\r\n");
        sb.Append("--").Append(Boundary).Append("\r\n");
        sb.Append(Encoding.UTF8.GetString(content)).Append("\r\n");
        sb.Append("--").Append(Boundary).Append("\r\n");
        sb.Append("Content-Type: application/x-pkcs7-signature; name=\"smime.p7s\"\r\n");
        sb.Append("Content-Transfer-Encoding: base64\r\n");
        sb.Append("Content-Disposition: attachment; filename=\"smime.p7s\"\r\n\r\n");
        sb.Append(signature).Append("\r\n\r\n");
        sb.Append("--").Append(Boundary).Append("--\r\n\r\n");
        return sb.ToString();
    }

    /// <summary>
    /// Verifies a signed document: the signature must be valid and its signer must chain to
    /// <paramref name="trustedCa"/>. Returns the signed document (without the MIME part header).
    /// </summary>
    public static string Verify(string smime, X509Certificate2 trustedCa)
    {
        var (content, signature) = Split(smime);
        var cms = new SignedCms(new ContentInfo(content), detached: true);
        cms.Decode(signature);
        cms.CheckSignature(verifySignatureOnly: true);

        foreach (var info in cms.SignerInfos)
        {
            var cert = info.Certificate ?? throw new CryptographicException("The signature does not carry the signer certificate.");
            if (!ChainsTo(cert, trustedCa))
            {
                throw new CryptographicException($"Signer '{cert.Subject}' is not issued by the trusted CA '{trustedCa.Subject}'.");
            }
        }

        var text = Encoding.UTF8.GetString(content);
        var body = text.IndexOf("\r\n\r\n", StringComparison.Ordinal);
        return (body >= 0 ? text[(body + 4)..] : text).Replace("\r\n", "\n");
    }

    /// <summary>True when <paramref name="cert"/> is <paramref name="ca"/> or chains to it.</summary>
    public static bool ChainsTo(X509Certificate2 cert, X509Certificate2 ca)
    {
        if (cert.RawData.AsSpan().SequenceEqual(ca.RawData))
        {
            return true;
        }

        using var chain = new X509Chain();
        chain.ChainPolicy.TrustMode = X509ChainTrustMode.CustomRootTrust;
        chain.ChainPolicy.CustomTrustStore.Add(ca);
        chain.ChainPolicy.RevocationMode = X509RevocationMode.NoCheck;
        chain.ChainPolicy.VerificationFlags = X509VerificationFlags.IgnoreWrongUsage;
        return chain.Build(cert);
    }

    /// <summary>The signed MIME part: a text/plain header, a blank line, then the document with CRLF line endings.</summary>
    private static byte[] Canonical(string document) =>
        Encoding.UTF8.GetBytes("Content-Type: text/plain\r\n\r\n" + document.Replace("\r\n", "\n").Replace("\n", "\r\n"));

    private static (byte[] Content, byte[] Signature) Split(string smime)
    {
        var text = smime.Replace("\r\n", "\n").Replace("\n", "\r\n");
        var bMarker = "boundary=\"";
        var bStart = text.IndexOf(bMarker, StringComparison.OrdinalIgnoreCase);
        if (bStart < 0)
        {
            throw new CryptographicException("Not an S/MIME multipart/signed document.");
        }

        bStart += bMarker.Length;
        var boundary = "--" + text[bStart..text.IndexOf('"', bStart)];
        var parts = text.Split(boundary);
        if (parts.Length < 3)
        {
            throw new CryptographicException("S/MIME document is missing its parts.");
        }

        // Part 1 is the signed content; the CRLF before each boundary belongs to the boundary.
        var content = parts[1];
        content = content.StartsWith("\r\n", StringComparison.Ordinal) ? content[2..] : content;
        content = content.EndsWith("\r\n", StringComparison.Ordinal) ? content[..^2] : content;

        var sigPart = parts[2];
        var sigBody = sigPart[(sigPart.IndexOf("\r\n\r\n", StringComparison.Ordinal) + 4)..];
        var signature = Convert.FromBase64String(new string(sigBody.Where(c => !char.IsWhiteSpace(c)).ToArray()));
        return (Encoding.UTF8.GetBytes(content), signature);
    }
}
