# ros2net — tool baris perintah

> ROS2.Net — dibuat oleh **Gravicode Studios** dipimpin oleh **Kang Fadhil**.

`ros2net` meniru CLI `ros2`, tetapi **tidak membutuhkan instalasi ROS 2**: ia berbicara DDS-RTPS
(middleware terkelola), rosbridge, atau middleware in-process.

```bash
dotnet tool install -g ROS2.Net.Cli
ros2net --help
```

## Opsi global

| Opsi | Arti |
|---|---|
| `--rmw rtps\|rosbridge\|inprocess` | Middleware (bawaan `ROS2NET_RMW`, kalau tidak ada `rtps`). |
| `--domain N` | ID domain ROS (bawaan `ROS_DOMAIN_ID`, kalau tidak ada 0). |
| `--url ws://host:9090` | URL rosbridge (otomatis memakai `--rmw rosbridge`). |
| `--localhost` | Discovery hanya di mesin ini. |
| `--spin-time S` | Lama menunggu discovery (detik) sebelum membaca graph (2 dtk RTPS, 1 dtk rosbridge, 0 in-process). |

Node pembantu milik CLI bersifat tersembunyi (`_ros2net_cli_*`) dan tidak muncul di `node list`.

## Perintah

| Perintah | Keterangan |
|---|---|
| `topic list [-t] [-c]` | Daftar topic (dengan tipe / jumlah). |
| `topic info <topic> [-v]` | Tipe, jumlah publisher dan subscription; `-v` menampilkan node, GID, dan QoS tiap endpoint. |
| `topic type <topic>` | Tipe topic. |
| `topic echo <topic> [tipe] [--once] [-n N] [--json] [--timeout S]` | Menampilkan pesan dalam YAML (atau satu baris JSON per pesan). Tipe dicari otomatis bila tidak diberikan; QoS menyesuaikan dengan publisher kecuali diberi `--qos-profile/--qos-reliability/--qos-durability/--qos-depth`. |
| `topic pub <topic> <tipe> '<yaml atau json>' [-r Hz] [--once] [-t N]` | Menerbitkan pesan, contoh `ros2net topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`. |
| `topic hz <topic>` / `topic bw <topic>` | Statistik laju / bandwidth tiap detik (`--window`, `--duration`). |
| `node list [-a] [-c]` / `node info <node>` | Daftar node; per node: subscriber, publisher, service server/client, action server/client. |
| `service list [-t]` / `service type <svc>` | Daftar service. |
| `service call <svc> [tipe] '<request>'` | Memanggil service; respons ditampilkan dalam YAML. |
| `action list [-t]` | Daftar action (dikenali dari service `_action/send_goal`). |
| `action send_goal <action> [tipe] '<goal>' [-f]` | Mengirim goal, menampilkan feedback (`-f`) dan hasilnya; Ctrl+C membatalkan goal. |
| `param list [node]` | Nama parameter (semua node bila node tidak disebut). |
| `param get <node> <nama>` / `param set <node> <nama> <nilai>` | Membaca / menulis parameter (`nilai` berformat YAML: `1`, `2.5`, `true`, `[1, 2]`). |
| `param describe <node> <nama...>` | Deskriptor (tipe, read-only, rentang). |
| `param dump <node> [-o file]` / `param load <node> <file>` | File YAML parameter ROS 2. |
| `interface list [-m\|-s\|-a]` / `interface package <pkg>` / `interface show <tipe>` | Interface yang sudah terkompilasi di ROS2.Net. |
| `interface generate <dir> -o <out>` | Membuat kode C# dari file `.msg/.srv/.action` (`<pkg>/<msg\|srv\|action>/Nama.ext`) tanpa source generator. |
| `bag record <topic...> \| -a \| -e regex [-o dir] [--compression zstd\|lz4] [-b byte] [-d detik] [--duration S]` | Merekam bag rosbag2 berformat MCAP. |
| `bag play <bag> [-r laju] [-l] [--clock [Hz]] [--topics ...] [-m lama:=baru] [--start-offset S]` | Memutar ulang bag. |
| `bag info <bag>` | Format sama dengan `ros2 bag info`. |
| `bag export <bag> --format jsonl\|csv [-t topic] [-o file]` | Dataset untuk pelatihan AI. |
| `security create_keystore <dir>` / `create_enclave <dir> <enclave>` / `list_enclaves <dir>` | Keystore SROS2. |
| `doctor [-r]` | Runtime, variabel lingkungan, middleware, antarmuka jaringan, uji multicast loopback, pustaka native. |
| `run-demo talker\|listener\|add_two_ints_server\|add_two_ints_client\|fibonacci_server [--duration S]` | Node demo bawaan. |

## Contoh

```bash
# Terminal 1                                  # Terminal 2
ros2net run-demo talker                       ros2net topic echo /chatter
ros2net run-demo add_two_ints_server          ros2net service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
ros2net run-demo fibonacci_server             ros2net action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" -f

ros2net param set /talker use_sim_time true
ros2net bag record -a -o run1 --compression zstd --duration 30
ros2net bag export run1 --format csv -t /odom -o odom.csv
ros2net topic list --url ws://robot.local:9090   # lewat rosbridge
```

> Di Git Bash (Windows), argumen yang diawali `/` diubah menjadi path Windows. Jalankan
> `export MSYS_NO_PATHCONV=1` terlebih dahulu, atau gunakan PowerShell/cmd.

## Keterbatasan

- `service call` dan `action send_goal` hanya bekerja untuk tipe interface yang terkompilasi di
  ROS2.Net (semua interface standar, lihat `interface list`); Core belum punya API service untuk
  tipe dinamis.
- `topic echo`/`pub` membutuhkan definisi tipe dari registry yang sama. Tipe yang hanya ada di sisi
  remote tidak dapat didekode.
- Interoperabilitas dengan graph ROS 2 asli (Fast DDS / Cyclone DDS) memakai middleware RTPS
  terkelola, yang sejauh ini baru diverifikasi antar proses ROS2.Net.
