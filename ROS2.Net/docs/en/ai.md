# AI integration (`ROS2.Net.AI`)

Reinforcement learning on the simulator, ONNX model inference on topics, natural-language robot commands through an LLM, and telemetry anomaly detection.

> Made by Gravicode Studios, led by Kang Fadhil.

## Reinforcement learning

`RobotGymEnv` is a point-goal navigation task over `SimWorld`. It implements RLNet's `IDiscreteEnvironment`, so every RLNet discrete agent can train on it: DQN, Q-table, PPO and A2C.

- **Observation** (`NavigationObservation`): `[sector_0 … sector_{n-1}, distance, cos(bearing), sin(bearing)]`. Each sector is the minimum LiDAR range in that slice, normalized to [0, 1].
- **Actions**: forward, forward-left, forward-right, rotate left, rotate right. `ActionCount` uses the first N.
- **Reward**: progress toward the goal × `ProgressScale`, minus a small cost per step, plus `GoalReward` on arrival and `CollisionPenalty` on contact.
- **Episode end**: the episode terminates on reaching the goal or on a collision, and is truncated at `MaxEpisodeSteps`.

```csharp
using ROS2.Net.AI.Gym;

var env = new RobotGymEnv(new RobotGymOptions { Scenario = ScenarioKind.Office, Sectors = 8 });
var agent = AgentFactory.CreateDqn(env, totalSteps: 100_000);
RlTrainer.Train(env, agent, episodes: 500, onEpisode: s => Console.WriteLine($"{s.Episode}: {s.Return:0.0}"));
var eval = RlTrainer.Evaluate(env, agent, episodes: 20);
```

`AgentFactory.CreateQTable(env, bins)` gives a tabular baseline. It learns small tasks (`Sectors = 0`, a fixed goal) in seconds, and a test checks that it beats a random policy.

**Deployment.** `PolicyRunnerNode(context, policy)` builds the same observation from `scan` and `odom`, then drives `cmd_vel` toward `goal_pose`. The policy can be an RLNet agent (`obs => agent.SelectAction(obs, true)`) or an `OnnxPolicy`. Spin the node; its control loop is a timer.

## ONNX inference

```csharp
using ROS2.Net.AI.Onnx;

using var runner = new OnnxModelRunner("detector.onnx");
float[] output = runner.Run(input);               // dynamic axes are resolved from the data length
var node = InferenceNode<Image, Detection2DArray>.ForOnnx(context, "detector", "/camera/image_raw", "/detections",
    runner, img => (Preprocess(img), new[] { 1, 3, 224, 224 }), (y, img) => Postprocess(y, img));
```

`InferenceNode` keeps only the newest input (keep-last 1), so latency stays bounded when the model is slower than the sensor. `OnnxPolicy.SelectAction` returns the arg-max of the model output.

## Natural-language commander

`NaturalLanguageCommanderNode` listens on `/nl_command` (`std_msgs/String`) and sends the text to an `IChatModel` along with four tools:

| Tool | Effect |
|---|---|
| `move_forward(distance_m, speed_mps?)` | A timed `cmd_vel` motion (negative distance drives backwards) |
| `rotate(angle_deg, angular_speed?)` | Rotates in place; positive angles turn left |
| `go_to_waypoint(name)` | Publishes `geometry_msgs/PoseStamped` on `/goal_pose` for Nav2 or `ROS2.Net.Navigation` |
| `stop()` | Publishes a zero twist |

Every value passes through the `SafetyLimiter`, which enforces maximum speed, distance and rotation. Forward motion also stops when the LiDAR sees an obstacle closer than `StopDistance` in the front ±30°. Motions always end with a zero twist, and a new command interrupts the running one. Feedback is published on `/nl_feedback`.

Chat models:

- **`OpenAICompatibleChatModel`**: any `/v1/chat/completions` server, such as Ollama (the default, `http://localhost:11434/v1`), llama.cpp server, LM Studio, vLLM, OpenAI or Azure OpenAI (set `UseAzureApiKeyHeader` and `QueryString`). Settings come from `ROS2NET_LLM_ENDPOINT`, `ROS2NET_LLM_MODEL` and `ROS2NET_LLM_API_KEY` (or `OPENAI_API_KEY`). No keys are stored in code.
- **`RuleBasedChatModel`**: works offline and deterministically. It understands simple English and Bahasa Indonesia ("forward 1 m then turn right 90", "maju 50 cm lalu belok kiri", "pergi ke dock", "berhenti"). Use it for demos without a server, or as a fallback.

```csharp
var commander = new NaturalLanguageCommanderNode(context, new OpenAICompatibleChatModel(), new CommanderOptions
{
    CmdVelTopic = "/robot1/cmd_vel",
    ScanTopic = "/robot1/scan",
    Waypoints = { ["dock"] = (2, 2, 0), ["shelf_A1"] = (6, 3, 1.57) },
});
```

## Anomaly detection

- `EwmaAnomalyDetector`: an exponentially weighted mean and variance with a z-score threshold. Anomalous samples are weighted down so they do not mask the next spike.
- `RollingZScoreDetector`: a z-score over a fixed sliding window.
- `TelemetryAnomalyMonitor<T>`: extracts named signals from a topic and publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics` (WARN on an anomaly), plus an `AnomalyDetected` event.

```csharp
using var monitor = new TelemetryAnomalyMonitor<BatteryState>(node, "/robot1/battery_state",
    [("current", b => b.Current), ("voltage", b => b.Voltage)]);
```

## Notes

- RLNet (`Gravicode.RLNet`) provides the agents. `RlTrainer` is a thin loop, so RLNet's own `Trainer` works too, because `RobotGymEnv` is a standard RLNet environment.
- ONNX Runtime ships native binaries for each platform through its NuGet package.
