# RTPS middleware (`ROS2.Net.Rmw.Rtps`)

`ROS2.Net.Rmw.Rtps` is a pure managed implementation of the OMG **DDS-RTPS 2.3** wire protocol. Its
identifier is `rtps`. It needs no native DDS library: a ROS2.Net process becomes a DDS domain
participant on its own, using the same discovery, ports, name mangling and graph topic as the stock
ROS 2 middlewares.

> Made by Gravicode Studios, led by Kang Fadhil.

## Quick start

```csharp
using ROS2.Net;
using ROS2.Net.Rmw.Rtps;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var ctx = await Ros2Context.CreateAsync(new Ros2ContextOptions { DomainId = 0 }.UseRtps());
var node = ctx.CreateNode("talker");
var pub = node.CreatePublisher<StdMsgs.String>("chatter");
await pub.WaitForSubscriptionsAsync(1, TimeSpan.FromSeconds(5));
pub.Publish(new StdMsgs.String { Data = "hello" });
```

You can also select the middleware without code changes. Set `ROS2NET_RMW=rtps`, or set
`RMW_IMPLEMENTATION` to `rmw_fastrtps_cpp` or `rmw_cyclonedds_cpp`. The package must be referenced and
loaded: touching any of its types, or calling `UseRtps()`, registers the factory.

## Options

Pass options in code with `UseRtps(o => ...)`, or as `key=value` strings in `Ros2ContextOptions.RmwSettings`:

| Option | Setting key | Default | Meaning |
|---|---|---|---|
| `LocalhostOnly` | `localhost` | `false` (also set by `ROS_LOCALHOST_ONLY=1`) | Bind to and announce only 127.0.0.1; no multicast. |
| `UseMulticast` | `multicast` | `true` | Send SPDP to 239.255.0.1 and listen on the multicast ports. |
| `InterfaceAddress` | `interface` | all up IPv4 NICs | Restrict traffic to one local address. |
| `InitialPeers` | `initial_peers` | — | Extra unicast discovery peers, `host` or `host:port`, comma separated. |
| `LocalPeerParticipantIds` | — | `10` | Same-host participants found by unicasting SPDP to participant ids 0..N-1 on 127.0.0.1. |
| `ParticipantLeaseDuration` | `lease_seconds` | 20 s | How long peers keep us without hearing an announcement. |
| `AnnouncementPeriod` | — | 3 s | SPDP period after the first five fast (200 ms) announcements. |
| `HeartbeatPeriod` | — | 100 ms | Reliable writer heartbeat period while samples are unacknowledged (backs off up to ×32). |
| `MaxFragmentSize` | `max_fragment_size` | 60000 | Larger payloads are sent as `DATA_FRAG`. |
| `ServiceMapping` | `service_mapping` | `FastDds` | How service replies are correlated with requests (see below). |
| `Distro` | `distro` / `gid_size` | `Jazzy` | Layout of `rmw_dds_common/msg/Gid` in `ros_discovery_info`. |
| `PublishDiscoveryInfo` | — | `true` | Publish and read `ros_discovery_info`. |

## What is implemented

- **Transport.** UDPv4 with the standard port mapping: PB 7400, DG 250, PG 2, d0 0, d1 10, d2 1,
  d3 11. Participant ids are picked automatically. There is one receive loop per socket, buffers come
  from the pool, and socket buffers are 4 MB so fragment bursts are not dropped.
- **Messages.** `INFO_TS`, `INFO_DST`, `INFO_SRC`, `DATA`, `DATA_FRAG`, `HEARTBEAT`, `ACKNACK`,
  `GAP` and `NACK_FRAG`, in both byte orders. `PL_CDR` parameter lists are supported. Payload padding
  is recorded in the encapsulation options.
- **SPDP.** Participant announcements over multicast and unicast peers carry `USER_DATA` with
  `enclave=…;` and leases. Lease expiry and dispose messages remove a participant.
- **SEDP.** Reliable, transient-local built-in publication and subscription writers and readers.
  Each announcement carries the topic, type name, reliability, durability, history, deadline,
  liveliness, lifespan and data representation (XCDR1). Endpoints are unregistered with
  `PID_STATUS_INFO`. Matching uses topic + type + QoS through `QosMatching`, and an incompatible
  match raises `RequestedIncompatibleQos` or `OfferedIncompatibleQos`.
- **Reliability.** Stateful writers keep a history bounded by QoS depth. They piggyback heartbeats on
  every sample, heartbeat periodically, and repair losses on `ACKNACK` (resend, or `GAP` for samples
  no longer held) and on `NACK_FRAG`. Readers deliver in order, buffer out-of-order samples, request
  what is missing and handle `GAP`. Best-effort readers keep the newest sample. Transient-local
  writers push their history to late joiners.
- **Endpoints in the same participant** match locally and are delivered directly.
  `IgnoreLocalPublications` is honoured, so intra-process comms in `Ros2Context` deliver each local
  message exactly once.
- **ROS 2 conventions.** Topics are `rt/<name>`. Services are `rq/<name>Request` and
  `rr/<name>Reply`. Type names follow the DDS form `pkg::msg::dds_::Name_`. With
  `AvoidRosNamespaceConventions`, the topic name is used verbatim.
- **Graph.** Nodes and endpoints are published on `ros_discovery_info`
  (`rmw_dds_common/msg/ParticipantEntitiesInfo`). Endpoints learned from SEDP are mapped back to ROS
  names and owning nodes; owners that cannot be resolved show as `_NODE_NAME_UNKNOWN_`, as in rclcpp.

### Services

- **`FastDds` (default).** The reply carries the request's sample identity, meaning the request
  writer GUID plus its sequence number, in inline QoS. It is sent under both `PID_RELATED_SAMPLE_IDENTITY`
  ids: `0x800f` (eProsima) and `0x0083` (DDS-RPC). Requests carry the client's reply reader GUID in the
  same parameter, as rmw_fastrtps does.
- **`CycloneDds`.** Request and reply payloads start with `{ uint64 client_guid; int64 sequence; }`
  inside the CDR stream, the layout of rmw_cyclonedds' `cdds_request_header_t`. The client id is a
  64-bit FNV-1a hash of the request writer GUID.

Both ends of a service must use the same mapping.

### `ros_discovery_info` and ROS distributions

`rmw_dds_common/msg/Gid` changed between distributions. Up to **Humble and Iron** it is `uint8[24]`
(the GUID followed by zero padding); from **Jazzy** it is `uint8[16]`. Set `Distro = RosDistro.Humble`
(or the setting `distro=humble`) when the other side runs Humble or Iron. A mismatch only affects
node names in the graph; topics, services and actions keep working.

## Limitations and verification status

- **Interop with Fast DDS and Cyclone DDS is not verified.** There was no ROS 2 installation on the
  machine where this was built, so the wire format was written from the RTPS 2.3 specification and
  public rmw sources, and tested ROS2.Net to ROS2.Net only. Please report interop results.
- XCDR1 only (no XCDR2 or type-object negotiation). No keyed user topics; ROS topics are keyless.
- No DDS Security (SROS2 governance and permissions) on the wire, no shared-memory transport, no
  IPv6, no WLP (liveliness is assumed automatic and infinite). `DATA_FRAG` reassembly is limited to
  256 MB per sample.
- No type-hash `USER_DATA` (`typehash=RIHS01_…`), which Jazzy uses only for introspection.

## Tests

```bash
dotnet build src/ROS2.Net.Rmw.Rtps
dotnet test tests/ROS2.Net.Rmw.Rtps.Tests
dotnet test tests/ROS2.Net.Rmw.Rtps.Tests --filter "FullyQualifiedName~Fibonacci"
```

Wire tests check encodings against bytes computed by hand from the spec. The integration tests run
two participants over loopback and cover:

- talker/listener
- a 1 MB fragmented point cloud
- 300 in-order reliable samples
- a transient-local late joiner
- services in both mappings
- a Fibonacci action
- graph discovery
- QoS incompatibility
- participant leave
- local matching and intra-process delivery
