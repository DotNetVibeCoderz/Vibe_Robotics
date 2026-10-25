# rosbag2 — merekam, memutar ulang dan membuat dataset

`ROS2.Net.Bag` merekam dan memutar ulang topik ROS 2 dalam format rosbag2 MCAP. Sebuah bag adalah direktori berisi `<nama>_0.mcap` (ditambah `_1`, `_2`, ... setelah dipecah) dan `metadata.yaml` rosbag2.

```bash
dotnet add package ROS2.Net.Bag
```

## Merekam

```csharp
var node = context.CreateNode("recorder");
await using var recorder = new BagRecorder(node, "bags/run1", new BagRecorderOptions
{
    All = true,                                    // atau Topics = { "/scan", "/odom" }
    Exclude = "^/(rosout|parameter_events)$",      // regex, seperti ros2 bag record -x
    Writer = new BagWriterOptions { Compression = McapCompression.Zstd, MaxFileSize = 512 << 20 },
});
recorder.Start();
// ...
await recorder.StopAsync();                        // menulis sisa data dan metadata.yaml
```

- Topik ditemukan lewat graph: diperiksa setiap `DiscoveryInterval`, dan lebih cepat bila graph berubah. Topik yang muncul belakangan ikut terekam secara otomatis. Sebuah topik baru direkam setelah ada publisher-nya. Gunakan `TopicTypes` untuk mulai merekam topik sebelum ada yang mempublikasikannya.
- Pesan diterima sebagai CDR mentah dan tidak pernah di-deserialisasi. Pesan disalin ke buffer dari pool lalu ditulis oleh satu task latar belakang, sehingga thread penerima middleware tidak pernah tertahan oleh I/O disk.
- **Adaptasi QoS** bekerja seperti rosbag2: subscription memakai reliable / transient-local hanya jika semua publisher menawarkannya, selain itu best-effort / volatile. Profil QoS yang ditawarkan publisher disimpan untuk tiap topik.
- **Pemecahan file:** `MaxFileSize` (byte) dan `MaxFileDuration`.

`BagWriter` adalah API tingkat rendah untuk menulis data yang sudah Anda miliki: `Write(topic, cdr, time)` atau `Write<T>(topic, message, time)`.

## Memutar ulang

```csharp
using var player = new BagPlayer(context.CreateNode("player"), "bags/run1", new BagPlayerOptions
{
    Rate = 2.0,
    Loop = false,
    StartOffset = TimeSpan.FromSeconds(5),
    ClockRate = 100,                               // publikasikan /clock untuk node use_sim_time
    Remappings = { ["/scan"] = "/scan_replay" },
});
await player.PlayAsync(ct);
```

- Jeda antar-pesan dipertahankan dan diskalakan dengan `Rate`, yang bisa diubah saat sedang diputar.
- `Pause()`, `Resume()` dan `TogglePause()` mengendalikan pemutaran. Selama dijeda, `StepAsync()` mempublikasikan tepat satu pesan.
- Setiap topik dipublikasikan dengan QoS saat direkam, sehingga topik *latched* seperti `/map` dan `/tf_static` tetap *latched*. `QosOverrides` mengganti QoS untuk topik tertentu.
- `Topics` menyaring topik yang diputar. `Delay` menunggu sebelum pesan pertama.

## Membaca dan memeriksa

```csharp
using var bag = BagReader.Open("bags/run1");      // direktori, atau satu file .mcap
foreach (var m in bag.ReadMessages(start, end, topics: ["/imu"]))
{
    var imu = BagReader.Deserialize<Imu>(m);      // bertipe
    string json = bag.ToJson(m);                  // memakai skema yang tersimpan di bag: tidak perlu tipe hasil generate
}

Console.WriteLine(BagInfo.Read("bags/run1"));     // tata letaknya sama dengan `ros2 bag info`
```

Pesan dikembalikan berurutan menurut waktu log. Pada file yang terindeks (semua file yang ditulis ROS2.Net terindeks), pembacaan rentang waktu hanya mendekompresi *chunk* yang beririsan dengan rentang tersebut.

## Dataset untuk pelatihan AI

```csharp
BagExporter.ToJsonLinesFile(bag, "imu.jsonl", ["/imu"]);    // {"topic","type","log_time_ns","publish_time_ns","message":{...}}
BagExporter.ToCsvFile(bag, "/imu", "imu.csv");              // kolom diratakan: angular_velocity.z, orientation_covariance[8], ...
```

Kolom CSV terdiri dari `log_time_ns` ditambah setiap field daun dari pesan pertama. Field bersarang ditulis sebagai `a.b` dan array sebagai `a[i]`, sampai batas `maxArrayElements`. Blob byte base64 seperti data gambar dilewati.

## Detail MCAP

`McapWriter` dan `McapReader` mengikuti [spesifikasi MCAP](https://mcap.dev/spec):
- Record: Header (profil `ros2`), Schema (`ros2msg`, berisi definisi lengkap dari `RosTypeRegistry.GetFullDefinition`), Channel (`cdr`, dengan metadata `offered_qos_profiles`), Message, Chunk, Message Index, Chunk Index, Statistics, Metadata, Metadata Index, Data End, Summary Offset dan Footer.
- **CRC32:** untuk chunk, bagian data dan bagian summary, semuanya diperiksa saat membaca.
- **Kompresi:** none, zstd (ZstdSharp) dan frame lz4 (K4os). Ketiganya kode managed murni.
- **File tanpa summary** dibaca dengan memindai berurutan.

`metadata.yaml` ditulis dalam format rosbag2 versi 5, dengan `offered_qos_profiles` dalam bentuk angka seperti Humble. Pembaca menerima bentuk angka maupun bentuk string Iron/Jazzy. `compression_format` di tingkat bag dibiarkan kosong karena kompresi chunk MCAP transparan bagi rosbag2.

## Status verifikasi

Pengujian mencakup hal-hal berikut:
- round trip dengan ketiga jenis kompresi
- magic dan tata letak file, deteksi CRC pada chunk yang rusak dan pembacaan rentang waktu berbasis indeks
- isi `metadata.yaml`, pemecahan file dan dekode JSON dari skema yang tersimpan
- ekspor JSONL dan CSV
- recorder → player secara utuh lewat middleware in-process pada kecepatan 2×, dengan remapping
- *step* saat dijeda dan `/clock`

**Belum diverifikasi:** bag ini belum dibuka dengan `ros2 bag` atau Foxglove Studio sungguhan, karena tidak ada instalasi ROS 2 di lingkungan build. Formatnya mengikuti spesifikasi, tetapi anggap kompatibilitas lintas-tool belum teruji.
