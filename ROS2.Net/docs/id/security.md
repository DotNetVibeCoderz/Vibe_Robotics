# Keamanan (SROS2)

`ROS2.Net.Security` membuat dan memeriksa keystore SROS2, yaitu artefak keamanan yang dimuat oleh
middleware DDS-Security (Fast DDS, Cyclone DDS, Connext). Keystore ini menyediakan autentikasi, kontrol
akses, dan enkripsi. Struktur hasilnya sama dengan keluaran `ros2 security create_keystore` /
`create_enclave`, jadi bisa langsung dipakai oleh node ROS 2 biasa.

> **Penting:** middleware RTPS managed di ROS2.Net **belum** mengimplementasikan DDS-Security pada level
> wire. Pakailah keystore ini untuk deployment ROS 2 yang berjalan di Fast DDS atau Cyclone DDS, dan
> anggap sebagai fondasi untuk transport aman ROS2.Net di masa depan. Middleware rosbridge sebaiknya
> diamankan di level transport (`wss://`, autentikasi rosbridge, atau VPN).

```csharp
using ROS2.Net.Security;

var keystore = SrosKeystore.Create("demo_keystore");              // CA + governance bertanda tangan
keystore.CreateEnclave("/talker_listener/talker");                 // izin allow-all

var policy = SecurityPolicy.Load("policy.xml");                   // format policy sros2
keystore.CreateEnclaves(policy);                                   // satu enclave per <enclave>

var check = keystore.VerifyEnclave("/talker_listener/talker");
Console.WriteLine(check.IsValid ? "ok" : string.Join("\n", check.Problems));
```

Menjalankan node ROS 2 biasa dengan keystore tersebut:

```bash
export ROS_SECURITY_KEYSTORE=$PWD/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```

## Struktur

```
keystore/
  public/   ca.cert.pem  identity_ca.cert.pem  permissions_ca.cert.pem
  private/  ca.key.pem   identity_ca.key.pem   permissions_ca.key.pem
  enclaves/ governance.xml  governance.p7s
            <path enclave>/ cert.pem key.pem identity_ca.cert.pem permissions_ca.cert.pem
                            governance.p7s permissions.xml permissions.p7s
```

- CA berupa sertifikat self-signed **ECDSA P-256** (default `CN=sros2testCA`, berlaku 10 tahun). sros2
  membuat symlink CA identity dan CA permissions ke CA ini; library ini menulis salinannya, sehingga juga
  berjalan di Windows.
- Identitas enclave berupa sertifikat P-256 dengan subject `CN=<enclave>` yang diterbitkan oleh CA.
- `governance.xml` mengikuti template sros2: domain 0–230, kontrol akses saat join, discovery dan
  liveliness terenkripsi, RTPS ditandatangani, serta setiap topic dikontrol baca/tulisnya dan dienkripsi.
- File `.p7s` adalah **tanda tangan S/MIME detached PKCS#7 (SHA-256)** yang dibuat dengan CA, sesuai
  format yang ditulis sros2.

## Dari policy menjadi permissions

`SecurityPolicy` membaca XML policy sros2 (`<policy version="0.2.0"><enclaves><enclave path>
<profiles><profile ns node>` dengan `<topics publish|subscribe>`, `<services reply|request>`, dan
`<actions execute|call>`). Policy juga bisa dibuat lewat kode (`new EnclavePolicy("/x").AllowPublish("/chatter")`).
Nama diekspansi relatif terhadap profile (`~` = namespace privat), lalu diubah ke nama topic DDS:

| ROS | Topic DDS |
|---|---|
| topic `/chatter` | `rt/chatter` |
| service `/reset` (server: `reply`) | subscribe `rq/resetRequest`, publish `rr/resetReply` |
| service `/reset` (klien: `request`) | publish `rq/resetRequest`, subscribe `rr/resetReply` |
| action `/fibonacci` | tiga service `_action/*` miliknya, ditambah `rt/fibonacci/_action/feedback` dan `status` |

Kecuali `IncludeDefaults` bernilai false, setiap enclave juga boleh memakai `ros_discovery_info`,
`/rosout`, `/parameter_events`, `/clock`, dan service parameter. Entri DENY menjadi `<deny_rule>`, dan
default grant-nya adalah DENY. Tanpa policy, `create_enclave` mengizinkan publish/subscribe untuk `*`,
sama seperti sros2.

## Verifikasi

`SmimeSigner.Verify(smime, ca)` memeriksa tanda tangan sekaligus memastikan penandatangannya berantai
ke CA, lalu mengembalikan dokumen yang ditandatangani. `SrosKeystore.VerifyEnclave` memeriksa
kelengkapan file, memastikan sertifikat identitas diterbitkan oleh CA dan memuat nama enclave, serta
memastikan kedua tanda tangan valid. Pengujiannya mencakup dokumen yang dimanipulasi dan tanda tangan
dari CA lain.

Interoperabilitas dengan Fast DDS dan Cyclone DDS saat memuat file-file ini **belum** diverifikasi di
mesin ini (tidak ada instalasi ROS 2). Formatnya mengikuti keluaran sros2.
