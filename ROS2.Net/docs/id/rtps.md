# Middleware RTPS (`ROS2.Net.Rmw.Rtps`)

`ROS2.Net.Rmw.Rtps` adalah implementasi protokol wire OMG **DDS-RTPS 2.3** yang sepenuhnya managed,
dengan identifier `rtps`. Tidak diperlukan library DDS native. Sebuah proses ROS2.Net langsung menjadi
participant domain DDS sendiri, dengan discovery, port, name mangling, dan topik graph yang sama seperti
middleware bawaan ROS 2.

> Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.

## Mulai cepat

```csharp
using ROS2.Net;
using ROS2.Net.Rmw.Rtps;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var ctx = await Ros2Context.CreateAsync(new Ros2ContextOptions { DomainId = 0 }.UseRtps());
var node = ctx.CreateNode("talker");
var pub = node.CreatePublisher<StdMsgs.String>("chatter");
await pub.WaitForSubscriptionsAsync(1, TimeSpan.FromSeconds(5));
pub.Publish(new StdMsgs.String { Data = "halo" });
```

Middleware ini juga bisa dipilih tanpa mengubah kode. Atur `ROS2NET_RMW=rtps`, atau atur
`RMW_IMPLEMENTATION` ke `rmw_fastrtps_cpp` atau `rmw_cyclonedds_cpp`. Package-nya harus direferensikan
dan sudah dimuat: factory-nya terdaftar begitu salah satu tipenya dipakai atau `UseRtps()` dipanggil.

## Opsi

Opsi bisa diberikan di kode lewat `UseRtps(o => ...)`, atau sebagai string `key=value` di
`Ros2ContextOptions.RmwSettings`:

| Opsi | Kunci setting | Default | Arti |
|---|---|---|---|
| `LocalhostOnly` | `localhost` | `false` (juga diaktifkan oleh `ROS_LOCALHOST_ONLY=1`) | Hanya bind dan mengumumkan 127.0.0.1, tanpa multicast. |
| `UseMulticast` | `multicast` | `true` | Mengirim SPDP ke 239.255.0.1 dan mendengarkan port multicast. |
| `InterfaceAddress` | `interface` | semua NIC IPv4 yang aktif | Membatasi trafik ke satu alamat lokal. |
| `InitialPeers` | `initial_peers` | — | Peer discovery unicast tambahan, `host` atau `host:port`, dipisahkan koma. |
| `LocalPeerParticipantIds` | — | `10` | Participant di host yang sama ditemukan lewat SPDP unicast ke participant id 0..N-1 di 127.0.0.1. |
| `ParticipantLeaseDuration` | `lease_seconds` | 20 detik | Berapa lama peer menganggap kita masih ada tanpa pengumuman baru. |
| `AnnouncementPeriod` | — | 3 detik | Periode SPDP setelah lima pengumuman cepat (200 ms) pertama. |
| `HeartbeatPeriod` | — | 100 ms | Periode heartbeat writer reliable selama masih ada sampel yang belum di-ack (melambat hingga ×32). |
| `MaxFragmentSize` | `max_fragment_size` | 60000 | Payload yang lebih besar dikirim sebagai `DATA_FRAG`. |
| `ServiceMapping` | `service_mapping` | `FastDds` | Cara reply service dikaitkan dengan request-nya (lihat di bawah). |
| `Distro` | `distro` / `gid_size` | `Jazzy` | Layout `rmw_dds_common/msg/Gid` di `ros_discovery_info`. |
| `PublishDiscoveryInfo` | — | `true` | Mem-publish dan membaca `ros_discovery_info`. |

## Yang sudah diimplementasikan

- **Transport.** UDPv4 dengan pemetaan port standar: PB 7400, DG 250, PG 2, d0 0, d1 10, d2 1,
  d3 11. Participant id dipilih otomatis. Ada satu receive loop per socket, buffer diambil dari pool,
  dan buffer socket 4 MB supaya ledakan fragmen tidak hilang.
- **Pesan.** `INFO_TS`, `INFO_DST`, `INFO_SRC`, `DATA`, `DATA_FRAG`, `HEARTBEAT`, `ACKNACK`, `GAP`,
  dan `NACK_FRAG`, dalam kedua urutan byte. Parameter list `PL_CDR` didukung. Padding payload dicatat
  di opsi encapsulation.
- **SPDP.** Pengumuman participant lewat multicast dan peer unicast membawa `USER_DATA` berisi
  `enclave=…;` beserta lease. Participant dihapus saat lease habis atau saat menerima pesan dispose.
- **SEDP.** Writer dan reader built-in untuk publikasi dan langganan, bersifat reliable dan
  transient-local. Setiap pengumuman membawa nama topik, nama tipe, reliability, durability, history,
  deadline, liveliness, lifespan, dan data representation (XCDR1). Endpoint di-unregister dengan
  `PID_STATUS_INFO`. Pencocokan berdasarkan topik + tipe + QoS lewat `QosMatching`. Pasangan yang tidak
  kompatibel memunculkan event `RequestedIncompatibleQos` atau `OfferedIncompatibleQos`.
- **Reliability.** Writer stateful menyimpan history yang dibatasi QoS depth. Heartbeat ikut dikirim
  bersama setiap sampel dan juga dikirim berkala. Kehilangan diperbaiki saat menerima `ACKNACK`
  (kirim ulang, atau `GAP` untuk sampel yang sudah tidak disimpan) dan `NACK_FRAG`. Reader
  mengirimkan sampel secara berurutan, menampung sampel yang datang tidak urut, meminta yang hilang,
  dan menangani `GAP`. Reader best-effort mengambil sampel terbaru. Writer transient-local langsung
  mengirim history-nya ke peserta yang bergabung belakangan.
- **Endpoint dalam participant yang sama** dicocokkan secara lokal dan dikirim langsung.
  `IgnoreLocalPublications` dihormati, sehingga intra-process comms di `Ros2Context` mengirim setiap
  pesan lokal tepat satu kali.
- **Konvensi ROS 2.** Topik menjadi `rt/<nama>`. Service menjadi `rq/<nama>Request` dan
  `rr/<nama>Reply`. Nama tipe memakai bentuk DDS `pkg::msg::dds_::Name_`. Dengan
  `AvoidRosNamespaceConventions`, nama topik dipakai apa adanya.
- **Graph.** Node dan endpoint di-publish ke `ros_discovery_info`
  (`rmw_dds_common/msg/ParticipantEntitiesInfo`). Endpoint dari SEDP dipetakan kembali ke nama ROS dan
  node pemiliknya. Pemilik yang tidak diketahui ditampilkan sebagai `_NODE_NAME_UNKNOWN_`, sama seperti
  di rclcpp.

### Service

- **`FastDds` (default).** Reply membawa sample identity dari request-nya, yaitu GUID writer request
  plus sequence number-nya, di inline QoS. Identity ini dikirim dengan kedua id
  `PID_RELATED_SAMPLE_IDENTITY`: `0x800f` (eProsima) dan `0x0083` (DDS-RPC). Request membawa GUID
  reader reply milik client di parameter yang sama, seperti yang dilakukan rmw_fastrtps.
- **`CycloneDds`.** Payload request dan reply diawali `{ uint64 client_guid; int64 sequence; }` di
  dalam stream CDR, sesuai layout `cdds_request_header_t` milik rmw_cyclonedds. Client id-nya adalah
  hash FNV-1a 64-bit dari GUID writer request.

Kedua sisi sebuah service harus memakai mapping yang sama.

### `ros_discovery_info` dan distribusi ROS

`rmw_dds_common/msg/Gid` berubah antar-distribusi. Sampai **Humble dan Iron** bentuknya `uint8[24]`
(GUID diikuti padding nol), sedangkan mulai **Jazzy** bentuknya `uint8[16]`. Atur
`Distro = RosDistro.Humble` (atau setting `distro=humble`) bila sisi lain menjalankan Humble atau Iron.
Ketidakcocokan hanya memengaruhi nama node di graph; topik, service, dan action tetap berjalan.

## Batasan dan status verifikasi

- **Interop dengan Fast DDS dan Cyclone DDS belum diverifikasi.** Mesin tempat library ini dibangun
  tidak memiliki instalasi ROS 2. Format wire ditulis dari spesifikasi RTPS 2.3 dan source rmw yang
  publik, lalu baru diuji antar-ROS2.Net. Laporan hasil interop sangat diharapkan.
- Hanya XCDR1 (belum ada XCDR2 atau negosiasi type object). Belum ada topik user dengan key; topik ROS
  memang tidak memakai key.
- Belum ada DDS Security (governance dan permissions SROS2) di level wire, transport shared memory,
  IPv6, maupun WLP (liveliness dianggap otomatis dan tak terbatas). Perakitan ulang `DATA_FRAG`
  dibatasi 256 MB per sampel.
- `USER_DATA` type hash (`typehash=RIHS01_…`) belum dikirim; di Jazzy ini hanya dipakai untuk
  introspeksi.

## Pengujian

```bash
dotnet build src/ROS2.Net.Rmw.Rtps
dotnet test tests/ROS2.Net.Rmw.Rtps.Tests
dotnet test tests/ROS2.Net.Rmw.Rtps.Tests --filter "FullyQualifiedName~Fibonacci"
```

Tes wire mencocokkan encoding dengan byte yang dihitung manual dari spesifikasi. Tes integrasi
menjalankan dua participant lewat loopback dan mencakup:

- talker/listener
- point cloud 1 MB yang difragmentasi
- 300 sampel reliable berurutan
- peserta transient-local yang bergabung belakangan
- service dengan kedua mapping
- action Fibonacci
- discovery graph
- QoS yang tidak kompatibel
- participant yang keluar
- pencocokan lokal dan pengiriman intra-process
