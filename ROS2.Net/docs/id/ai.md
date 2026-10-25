# Integrasi AI (`ROS2.Net.AI`)

Reinforcement learning di atas simulator, inferensi model ONNX pada topik, perintah robot dengan bahasa alami lewat LLM, dan deteksi anomali telemetri.

> Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.

## Reinforcement learning

`RobotGymEnv` adalah tugas navigasi menuju titik tujuan di atas `SimWorld`. Kelas ini mengimplementasikan `IDiscreteEnvironment` milik RLNet, sehingga semua agen diskrit RLNet bisa dilatih dengannya: DQN, Q-table, PPO dan A2C.

- **Observasi** (`NavigationObservation`): `[sektor_0 … sektor_{n-1}, jarak, cos(bearing), sin(bearing)]`. Tiap sektor berisi jarak LiDAR terdekat di irisan tersebut, dinormalisasi ke [0, 1].
- **Aksi**: maju, maju-kiri, maju-kanan, putar kiri, putar kanan. `ActionCount` memakai N aksi pertama.
- **Reward**: kemajuan menuju tujuan × `ProgressScale`, dikurangi biaya kecil tiap langkah, ditambah `GoalReward` saat tiba dan `CollisionPenalty` saat bertabrakan.
- **Akhir episode**: episode berakhir saat tujuan tercapai atau terjadi tabrakan, dan dipotong (truncated) pada `MaxEpisodeSteps`.

```csharp
using ROS2.Net.AI.Gym;

var env = new RobotGymEnv(new RobotGymOptions { Scenario = ScenarioKind.Office, Sectors = 8 });
var agent = AgentFactory.CreateDqn(env, totalSteps: 100_000);
RlTrainer.Train(env, agent, episodes: 500, onEpisode: s => Console.WriteLine($"{s.Episode}: {s.Return:0.0}"));
var eval = RlTrainer.Evaluate(env, agent, episodes: 20);
```

`AgentFactory.CreateQTable(env, bins)` menyediakan baseline tabular. Agen ini mampu mempelajari tugas kecil (`Sectors = 0`, tujuan tetap) dalam hitungan detik, dan sebuah test memastikan hasilnya mengalahkan kebijakan acak.

**Deployment.** `PolicyRunnerNode(context, policy)` menyusun observasi yang sama dari `scan` dan `odom`, lalu menggerakkan `cmd_vel` menuju `goal_pose`. Kebijakannya bisa berupa agen RLNet (`obs => agent.SelectAction(obs, true)`) atau `OnnxPolicy`. Lakukan spin pada node; loop kendalinya berupa timer.

## Inferensi ONNX

```csharp
using ROS2.Net.AI.Onnx;

using var runner = new OnnxModelRunner("detector.onnx");
float[] output = runner.Run(input);               // sumbu dinamis ditentukan dari panjang data
var node = InferenceNode<Image, Detection2DArray>.ForOnnx(context, "detector", "/camera/image_raw", "/detections",
    runner, img => (Preprocess(img), new[] { 1, 3, 224, 224 }), (y, img) => Postprocess(y, img));
```

`InferenceNode` hanya menyimpan input terbaru (keep-last 1), sehingga latensi tetap terkendali walaupun model lebih lambat daripada sensor. `OnnxPolicy.SelectAction` mengembalikan arg-max dari keluaran model.

## Komandan bahasa alami

`NaturalLanguageCommanderNode` mendengarkan `/nl_command` (`std_msgs/String`) dan mengirim teksnya ke sebuah `IChatModel`, beserta empat tool:

| Tool | Efek |
|---|---|
| `move_forward(distance_m, speed_mps?)` | Gerakan `cmd_vel` berwaktu (jarak negatif berarti mundur) |
| `rotate(angle_deg, angular_speed?)` | Berputar di tempat; sudut positif berarti belok kiri |
| `go_to_waypoint(name)` | Memublikasikan `geometry_msgs/PoseStamped` di `/goal_pose` untuk Nav2 atau `ROS2.Net.Navigation` |
| `stop()` | Memublikasikan twist nol |

Semua nilai melewati `SafetyLimiter`, yang menegakkan batas kecepatan, jarak dan rotasi. Gerakan maju juga dihentikan bila LiDAR mendeteksi rintangan lebih dekat dari `StopDistance` di sektor depan ±30°. Setiap gerakan selalu diakhiri twist nol, dan perintah baru menyela gerakan yang sedang berjalan. Umpan balik dipublikasikan di `/nl_feedback`.

Model chat:

- **`OpenAICompatibleChatModel`**: server `/v1/chat/completions` apa pun, misalnya Ollama (default, `http://localhost:11434/v1`), llama.cpp server, LM Studio, vLLM, OpenAI atau Azure OpenAI (atur `UseAzureApiKeyHeader` dan `QueryString`). Pengaturan diambil dari `ROS2NET_LLM_ENDPOINT`, `ROS2NET_LLM_MODEL` dan `ROS2NET_LLM_API_KEY` (atau `OPENAI_API_KEY`). Tidak ada kunci yang disimpan di kode.
- **`RuleBasedChatModel`**: berjalan offline dan deterministik. Model ini memahami perintah sederhana dalam bahasa Inggris dan Indonesia ("forward 1 m then turn right 90", "maju 50 cm lalu belok kiri", "pergi ke dock", "berhenti"). Gunakan untuk demo tanpa server, atau sebagai cadangan.

```csharp
var commander = new NaturalLanguageCommanderNode(context, new OpenAICompatibleChatModel(), new CommanderOptions
{
    CmdVelTopic = "/robot1/cmd_vel",
    ScanTopic = "/robot1/scan",
    Waypoints = { ["dock"] = (2, 2, 0), ["shelf_A1"] = (6, 3, 1.57) },
});
```

## Deteksi anomali

- `EwmaAnomalyDetector`: rata-rata dan varians berbobot eksponensial dengan ambang z-score. Sampel anomali diberi bobot lebih kecil agar tidak menutupi lonjakan berikutnya.
- `RollingZScoreDetector`: z-score atas jendela geser berukuran tetap.
- `TelemetryAnomalyMonitor<T>`: mengambil sinyal bernama dari sebuah topik dan memublikasikan `diagnostic_msgs/DiagnosticArray` di `/diagnostics` (WARN saat terjadi anomali), ditambah event `AnomalyDetected`.

```csharp
using var monitor = new TelemetryAnomalyMonitor<BatteryState>(node, "/robot1/battery_state",
    [("current", b => b.Current), ("voltage", b => b.Voltage)]);
```

## Catatan

- Agen disediakan oleh RLNet (`Gravicode.RLNet`). `RlTrainer` hanyalah loop tipis, jadi `Trainer` bawaan RLNet juga bisa dipakai, karena `RobotGymEnv` adalah environment RLNet standar.
- ONNX Runtime menyertakan binary native untuk tiap platform melalui paket NuGet-nya.
