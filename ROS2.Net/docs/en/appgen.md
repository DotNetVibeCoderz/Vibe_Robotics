# ROS2.Net App Generator (ROS2AppGen)

> Made by **Gravicode Studios**, led by **Kang Fadhil**.

ROS2AppGen is a desktop code editor (Avalonia) with an AI pair programmer, **Jack — The Code Bender**,
that writes complete ROS2.Net applications from a prompt: nodes, UI, simulation, tests. Jack reads and
edits the open project, builds and runs it, fixes errors, searches the web and knows the ROS2.Net API.

```bash
dotnet run --project apps/ROS2.Net.AppGen          # or: ROS2AppGen.exe [project folder]
```

## User interface

![ROS2AppGen with a project open and Jack's reply](../images/appgen/appgen-main.png)

| Area | What it does |
|---|---|
| **Explorer** (left) | VS Code-style tree of the project with file-type badges and indent guides. Right-click for new file/folder, rename, delete, copy path and reveal in the file manager. Follows the active file and refreshes when Jack changes files. |
| **Editor** (center) | AvaloniaEdit with TextMate highlighting (C#, XAML/AXAML, csproj, JSON, YAML, Markdown, Razor…), tabs with an unsaved-changes dot, line numbers (toggle), word wrap, zoom, find/replace (Ctrl+F), go to line (Ctrl+G) and format (Shift+Alt+F). Files Jack rewrites reload in place unless you have unsaved edits. |
| **Jack** (right) | Model picker at the top (every profile from Settings), clear thread, hide. Messages render Markdown; code blocks have **Copy** and **Insert** (puts the code at the editor cursor). Each tool Jack runs appears as a live row with its duration. Attach images with the paperclip or paste them (Ctrl+V); **Ctrl+Enter** sends, **Esc** stops. Drag the splitter to resize. |
| **Logs** (bottom) | *Output* (build, run, deploy) and *Jack · tools* (every tool call and its result). Ctrl+J shows/hides it. |
| **Status bar** | Project, file (● when unsaved), cursor, run target (amber **ROBOT** pill for a physical robot), build state, active model and what Jack is doing. |

### Menus, toolbar and shortcuts

| Command | Shortcut |
|---|---|
| New project (template gallery / blank) | Ctrl+Shift+N |
| Open project · open file | Ctrl+O · Ctrl+Shift+O |
| Save · save all · close file | Ctrl+S · Ctrl+Shift+S · Ctrl+W |
| Close project · Exit | File menu · Alt+F4 |
| Go to line · format code · find | Ctrl+G · Shift+Alt+F · Ctrl+F |
| Build · run on the active target · stop | Ctrl+Shift+B · F5 · Shift+F5 |
| Run in Simulator · Run on Robot · Manage run targets… | Build menu |
| Deploy (dotnet publish, choose runtime, self-contained) | Build menu / toolbar |
| Explorer · Jack · logs panels | Ctrl+B · Ctrl+L · Ctrl+J |
| Ask Jack · settings | Ctrl+I · Ctrl+, |
| Line numbers · word wrap · zoom | View menu · Alt+Z · Ctrl+= / Ctrl+- |

The **Jack** menu also has *Explain the selection* and *Fix the last build errors* (sends the
compiler errors to Jack, who edits the code and rebuilds).

### New project

![Template gallery](../images/appgen/appgen-new-project.png)

Choose **From template** — templates grouped by category (simulator, science, housekeeping,
entertainment, work/office, industry, education, disaster & safety, tools…) with search — or **Blank**
(console node, Avalonia desktop app or Blazor web app). The project opens with its entry point shown.

### Settings

![Settings](../images/appgen/appgen-settings.png)

Everything is stored in **App.config** next to the executable (created from defaults on first run;
`App.config.template` shows the format without secrets) and editable here:

- **Models** — profiles for OpenAI, Azure OpenAI, Claude, Gemini, Ollama and DeepSeek: name, provider,
  model, API key (masked, *Show* to reveal), endpoint, temperature, max tokens, image support, extra HTTP
  headers. *Use for Jack* makes a profile active; *Test connection* sends a tiny prompt.
- **Run targets** — simulators and physical robots Run can use (see below).
- **Jack** — system prompt, tool rounds per message, auto build/run.
- **Web tools** — Tavily API key for internet search.
- **Projects** — projects folder, ROS2.Net package source (nuget.org or a local feed) and version.
- **Editor** and **Layout** — line numbers, wrap, font size, theme, panel visibility and widths.
  Panel widths and visibility are also saved when you exit.

### Running in the simulator or on a robot

The dropdown next to **Run** picks the run target; its icon shows a screen for a simulator and a robot
for physical hardware. **F5** runs on the active target; the Build menu also has *Run in Simulator*,
*Run on Robot* and *Manage run targets…*.

![Run targets](../images/appgen/appgen-run-targets.png)

**Settings → Run targets** adds, edits and removes targets: kind (simulator or physical robot),
middleware (`inprocess` for simulators only, `rtps`, `rosbridge`), DDS domain, network interface,
localhost-only discovery, rosbridge URL, robot namespace, simulator scenario (Warehouse, Office, Maze,
CorridorLoop, AgricultureRows, Empty), linear/angular speed limits and extra environment variables. The
defaults are *Simulator (in-process)*, *Simulator (RTPS on localhost, visible to ros2 CLI)*, *Robot over
DDS/RTPS (LAN)* and *Robot via rosbridge*.

![Safety confirmation](../images/appgen/appgen-robot-safety.png)

The first run on a physical robot in a session asks for confirmation, showing the target, connection,
namespace and speed limits, with a reminder to clear the area and keep the e-stop ready. When Run
starts, the Output log prints the target and the environment variables passed to the app
(`ROS2NET_TARGET`, `ROS2NET_RMW`, `ROS_DOMAIN_ID`, …); values whose names look like secrets are masked.

### Light theme

![Light theme](../images/appgen/appgen-light.png)

The sun/moon button (or View → Light/Dark theme) switches themes; the choice is saved.

## Models and providers

Jack runs on Semantic Kernel. Each **profile** in Settings picks a provider, model, key and endpoint:

| Provider | How it connects | Default model | Notes |
|---|---|---|---|
| Claude | SK OpenAI connector → Anthropic's OpenAI-compatible endpoint `https://api.anthropic.com/v1/` | `claude-sonnet-5-5` | Keys that are not workspace-scoped need the header `anthropic-workspace-id: wrkspc_…` in *Extra headers*. Claude 5 models reject `temperature`, so it is omitted for them. |
| OpenAI | SK OpenAI connector | `gpt-5-mini` | GPT-5 / o-series ignore temperature and max tokens. |
| Azure OpenAI | SK Azure OpenAI connector (endpoint + deployment name as model) | `gpt-5-mini` | Endpoint like `https://<resource>.openai.azure.com/`. |
| Gemini | SK Google connector (v1beta) | `gemini-2.5-flash` | |
| Ollama | SK Ollama connector | `llama3.2` at `http://localhost:11434` | No key; choose a tool-calling model. |
| DeepSeek | SK OpenAI connector → `https://api.deepseek.com` | `deepseek-v4-flash` | No image input. |

API keys can be written as `env:VARIABLE_NAME` so App.config stays free of secrets. Keys are masked in
logs. `App.config` is git-ignored; ship `App.config.template` (defaults only).

## Jack's tools (kernel functions)

| Plugin | Functions |
|---|---|
| `project` | `list_files`, `read_file` (optional line range), `write_file`, `replace_in_file`, `append_to_file`, `delete_path`, `rename_path`, `create_folder`, `search_in_files` (regex), `get_active_file`, `get_selection`, `get_project_info` — every path is confined to the open project |
| `build` | `build_project` (parsed errors/warnings), `run_project_briefly` (always the simulator target), `add_package`, `get_run_targets`, `list_templates`, `get_template_source`, `create_project`, `create_project_from_template` |
| `ros2net` | `get_api_reference` (the ROS2.Net docs, embedded at build time), `get_type_api` (exact reflected signatures of every ROS2.Net type), `list_message_types`, `get_message_type` (fields, C# property names, constants, server/client snippets), `get_packages`, `list_reference_topics` |
| `web` | `search_internet` (Tavily), `scrape_webpage` (readable text; local/private addresses are refused) |
| `nuget` | `search_nuget` |
| `math` | `calculate` (safe evaluator: `+ − * / % ^`, trig, logs, `min/max/round/hypot/clamp`, `deg/rad`), `convert_units` |
| `time` | `now`, `convert_timezone`, `date_diff`, `add_duration` |

There is deliberately no shell tool. Jack's working rules are in its system prompt: inspect the project, look up the API, write files,
build until green, summarize. Each user message has a tool budget (`MaxToolRounds`, also the model round-trip limit).

## Run targets: simulator or physical robot

**Run** sends the app to the active **run target**. Four are created on first run:

| Target | Kind | Middleware |
|---|---|---|
| Simulator (in-process) | Simulator | `inprocess` |
| Simulator (RTPS on localhost, visible to ros2 CLI) | Simulator | `rtps`, localhost only |
| Robot over DDS/RTPS (LAN) | PhysicalRobot | `rtps`, limits 0.3 m/s · 1.0 rad/s |
| Robot via rosbridge | PhysicalRobot | `rosbridge` `ws://<robot-ip>:9090` |

A target reaches the app as environment variables: `ROS2NET_TARGET=sim|robot`, `ROS2NET_RMW`,
`ROS_DOMAIN_ID`, `ROS2NET_ROSBRIDGE_URL`, `ROS2NET_ROBOT_NS` (also passed as `--ros-args -r __ns:=…`),
`ROS_LOCALHOST_ONLY`, `ROS2NET_NETWORK_INTERFACE`, `ROS2NET_SIM_SCENARIO`, `ROS2NET_MAX_LINEAR`,
`ROS2NET_MAX_ANGULAR` and any extra variables you add.

Every generated project contains **`AppTarget.cs`**, which reads them:

```csharp
await using var context = await AppTarget.CreateContextAsync(args);          // middleware per target
using var sim = AppTarget.StartSimulator(context, ScenarioKind.Office);      // null on a real robot
var cmd = node.CreatePublisher<Twist>(AppTarget.Topic("cmd_vel"));          // namespaced topic
cmd.Publish(AppTarget.Limit(twist));                                         // SAFETY: velocity limits
```

In simulator mode `StartSimulator` builds the scenario and runs a `SimulationNode`, so `cmd_vel`, `odom`,
`scan`, `/map` and `/tf` exist. In physical-robot mode no simulator starts; the app uses the robot's own
topics. Templates that are lock-step simulation studies (maze, soccer, AGV traffic, coverage, crop rows,
SLAM study, turtle art) refuse to run against a physical robot. Navigation templates use odometry poses
and the robot's `/map` on real hardware. **Real robots:** keep the limits low, an emergency stop in reach,
and test in open space first.

## Templates

38 templates, each a complete, building app that uses the actual ROS2.Net APIs:

| Category | Templates |
|---|---|
| Simulator | Warehouse AMR fleet simulator · Turtlesim playground · Robot simulator dashboard (desktop) · Maze escape wall-follower |
| Science | Lab sample sorter arm · Sensor data recorder & exporter · Environmental anomaly lab |
| Housekeeping | Vacuum robot coverage planner · Battery & docking behavior tree · Smart home robot monitor (web) |
| Entertainment | Robot soccer referee · Robot dance choreographer · Turtle art studio |
| Work | Office delivery robot · Natural-language robot commander |
| Office | Document courier (desktop) · Reception visitor greeter |
| Industry | Conveyor PLC bridge · Pick-and-place work cell · Machine health monitor · AGV traffic controller |
| Education | ROS 2 basics course · Turtlesim classroom · PID tuning lab · Reinforcement learning navigation lab |
| Disaster & Safety | Search & rescue SLAM explorer · Fire detection patrol · Gas leak sensor network |
| Agriculture | Crop row follower · Greenhouse sensor dashboard (web) |
| Healthcare | Hospital medicine delivery · Patient vitals monitor (web) |
| Logistics | Order picking route planner · Fleet operations dashboard (web) |
| Tools | ROS topic inspector (desktop) · Bag data exporter · TF tree inspector · Parameter configuration tool |

Templates live in `src/ROS2.Net.AppGen.Core/Templates/<id>/` (`template.json` + files with
`{{ProjectName}}`, `{{Namespace}}`, `{{Ros2NetVersion}}`). The csproj, the Avalonia `App`/`Program` and
the Blazor host files are generated around them. When **Package source** points to a folder, projects get a
`nuget.config` with that local feed (the ROS2.Net packages are not on nuget.org yet; produce them with
`dotnet pack` into `artifacts/packages`).

## Tests

```bash
dotnet test tests/ROS2.Net.AppGen.Tests                                           # unit tests (no network)
ROS2NET_APPGEN_TEMPLATES=1 dotnet test tests/ROS2.Net.AppGen.Tests --filter Category=TemplateBuild   # build every template
ROS2NET_APPGEN_LIVE=1 dotnet test tests/ROS2.Net.AppGen.Tests --filter Category=RealLlm             # real providers
```

Live tests read keys at run time from a local key file outside the repository
(`ROS2NET_APPGEN_KEYFILE`); nothing is stored or printed. After changing a library's public API, refresh
Jack's API reference with a Release build and `ROS2NET_APPGEN_REFRESH_API=1`.
