# ros2net — command-line tool

> ROS2.Net — made by **Gravicode Studios**, led by **Kang Fadhil**.

`ros2net` mirrors the `ros2` CLI, but needs **no ROS 2 installation**: it speaks DDS-RTPS
(the managed middleware), rosbridge, or the in-process middleware.

```bash
dotnet tool install -g ROS2.Net.Cli
ros2net --help
```

## Global options

| Option | Meaning |
|---|---|
| `--rmw rtps\|rosbridge\|inprocess` | Middleware (default `ROS2NET_RMW`, else `rtps`). |
| `--domain N` | ROS domain id (default `ROS_DOMAIN_ID`, else 0). |
| `--url ws://host:9090` | rosbridge URL (implies `--rmw rosbridge`). |
| `--localhost` | Discover only on this machine. |
| `--spin-time S` | Seconds to wait for discovery before reading the graph (2 s RTPS, 1 s rosbridge, 0 in-process). |

The CLI's own helper node is hidden (`_ros2net_cli_*`) and is not shown by `node list`.

## Commands

| Command | Description |
|---|---|
| `topic list [-t] [-c]` | Topics (with types / count). |
| `topic info <topic> [-v]` | Type, publisher and subscription counts; `-v` prints node, GID and QoS per endpoint. |
| `topic type <topic>` | The topic's type. |
| `topic echo <topic> [type] [--once] [-n N] [--json] [--timeout S]` | Print messages as YAML (or one JSON line each). The type is discovered when omitted; QoS adapts to the publishers unless `--qos-profile/--qos-reliability/--qos-durability/--qos-depth` is given. |
| `topic pub <topic> <type> '<yaml or json>' [-r Hz] [--once] [-t N]` | Publish, e.g. `ros2net topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`. |
| `topic hz <topic>` / `topic bw <topic>` | Rate / bandwidth statistics every second (`--window`, `--duration`). |
| `node list [-a] [-c]` / `node info <node>` | Nodes; per node: subscribers, publishers, service servers/clients, action servers/clients. |
| `service list [-t]` / `service type <svc>` | Services. |
| `service call <svc> [type] '<request>'` | Call a service; prints the response as YAML. |
| `action list [-t]` | Actions (found from their `_action/send_goal` services). |
| `action send_goal <action> [type] '<goal>' [-f]` | Send a goal, print feedback (`-f`) and the result; Ctrl+C cancels the goal. |
| `param list [node]` | Parameter names (all nodes when omitted). |
| `param get <node> <name>` / `param set <node> <name> <value>` | Read / write a parameter (`value` is YAML: `1`, `2.5`, `true`, `[1, 2]`). |
| `param describe <node> <names...>` | Descriptors (type, read-only, ranges). |
| `param dump <node> [-o file]` / `param load <node> <file>` | ROS 2 parameter YAML files. |
| `interface list [-m\|-s\|-a]` / `interface package <pkg>` / `interface show <type>` | Interfaces compiled into ROS2.Net. |
| `interface generate <dir> -o <out>` | Generate C# from `.msg/.srv/.action` files (`<pkg>/<msg\|srv\|action>/Name.ext`) without the source generator. |
| `bag record <topics...> \| -a \| -e regex [-o dir] [--compression zstd\|lz4] [-b bytes] [-d seconds] [--duration S]` | Record an MCAP rosbag2 bag. |
| `bag play <bag> [-r rate] [-l] [--clock [Hz]] [--topics ...] [-m old:=new] [--start-offset S]` | Play back. |
| `bag info <bag>` | Same layout as `ros2 bag info`. |
| `bag export <bag> --format jsonl\|csv [-t topic] [-o file]` | Datasets for AI training. |
| `security create_keystore <dir>` / `create_enclave <dir> <enclave>` / `list_enclaves <dir>` | SROS2 keystores. |
| `doctor [-r]` | Runtime, environment variables, middlewares, network interfaces, multicast loopback test, native library. |
| `run-demo talker\|listener\|add_two_ints_server\|add_two_ints_client\|fibonacci_server [--duration S]` | Built-in demo nodes. |

## Examples

```bash
# Terminal 1                                  # Terminal 2
ros2net run-demo talker                       ros2net topic echo /chatter
ros2net run-demo add_two_ints_server          ros2net service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
ros2net run-demo fibonacci_server             ros2net action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" -f

ros2net param set /talker use_sim_time true
ros2net bag record -a -o run1 --compression zstd --duration 30
ros2net bag export run1 --format csv -t /odom -o odom.csv
ros2net topic list --url ws://robot.local:9090   # through rosbridge
```

> On Git Bash (Windows), arguments that start with `/` are rewritten to Windows paths. Run
> `export MSYS_NO_PATHCONV=1` first, or use PowerShell/cmd.

## Limitations

- `service call` and `action send_goal` work for interface types compiled into ROS2.Net (all
  standard interfaces, see `interface list`); Core has no dynamic-type service API yet.
- `topic echo`/`pub` need the type's definition, which comes from the same registry. Types that
  only exist on the remote side cannot be decoded.
- Interop with a real ROS 2 graph (Fast DDS / Cyclone DDS) uses the managed RTPS middleware, which
  has been verified only between ROS2.Net processes so far.
