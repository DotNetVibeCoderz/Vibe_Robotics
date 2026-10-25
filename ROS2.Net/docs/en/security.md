# Security (SROS2)

`ROS2.Net.Security` creates and checks SROS2 keystores, the security artifacts that DDS-Security
middlewares (Fast DDS, Cyclone DDS, Connext) load. They provide authentication, access control and
encryption. The output uses the same layout `ros2 security create_keystore` / `create_enclave` produce,
so it works with stock ROS 2 nodes.

> **Important:** the managed RTPS middleware in ROS2.Net does **not** implement DDS-Security on the
> wire yet. Use these keystores with ROS 2 deployments running Fast DDS or Cyclone DDS, and treat them as
> groundwork for future secure transport in ROS2.Net. The rosbridge middleware should be secured at the
> transport level instead (`wss://`, rosbridge authentication, a VPN).

```csharp
using ROS2.Net.Security;

var keystore = SrosKeystore.Create("demo_keystore");              // CA + signed governance
keystore.CreateEnclave("/talker_listener/talker");                 // allow-all permissions

var policy = SecurityPolicy.Load("policy.xml");                   // sros2 policy format
keystore.CreateEnclaves(policy);                                   // one enclave per <enclave>

var check = keystore.VerifyEnclave("/talker_listener/talker");
Console.WriteLine(check.IsValid ? "ok" : string.Join("\n", check.Problems));
```

Run a stock ROS 2 node with it:

```bash
export ROS_SECURITY_KEYSTORE=$PWD/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```

## Layout

```
keystore/
  public/   ca.cert.pem  identity_ca.cert.pem  permissions_ca.cert.pem
  private/  ca.key.pem   identity_ca.key.pem   permissions_ca.key.pem
  enclaves/ governance.xml  governance.p7s
            <enclave path>/ cert.pem key.pem identity_ca.cert.pem permissions_ca.cert.pem
                            governance.p7s permissions.xml permissions.p7s
```

- The CA is a self-signed **ECDSA P-256** certificate (`CN=sros2testCA` by default, 10 years).
  sros2 symlinks the identity and permissions CAs to it; this library writes copies, which also works on Windows.
- Enclave identities are P-256 certificates with subject `CN=<enclave>`, issued by the CA.
- `governance.xml` matches sros2's template: domains 0–230, join access control, discovery and liveliness
  encrypted, RTPS signed, every topic read/write controlled and encrypted.
- `.p7s` files are **S/MIME detached PKCS#7 signatures (SHA-256)** made with the CA, the format sros2 writes.

## Policies → permissions

`SecurityPolicy` reads the sros2 policy XML (`<policy version="0.2.0"><enclaves><enclave path>
<profiles><profile ns node>` with `<topics publish|subscribe>`, `<services reply|request>` and
`<actions execute|call>`). It can also be built in code (`new EnclavePolicy("/x").AllowPublish("/chatter")`).
Names are expanded relative to the profile (`~` = private namespace) and mangled to DDS topics:

| ROS | DDS topic |
|---|---|
| topic `/chatter` | `rt/chatter` |
| service `/reset` (server: `reply`) | subscribe `rq/resetRequest`, publish `rr/resetReply` |
| service `/reset` (client: `request`) | publish `rq/resetRequest`, subscribe `rr/resetReply` |
| action `/fibonacci` | its three `_action/*` services plus `rt/fibonacci/_action/feedback` and `status` |

Unless `IncludeDefaults` is false, every enclave may also use `ros_discovery_info`, `/rosout`,
`/parameter_events`, `/clock` and the parameter services. DENY entries become a `<deny_rule>`, and the
grant's default is DENY. Without a policy, `create_enclave` grants publish/subscribe on `*`, as sros2 does.

## Verification

`SmimeSigner.Verify(smime, ca)` checks the signature and that the signer chains to the CA, then returns
the signed document. `SrosKeystore.VerifyEnclave` checks that the files exist, that the identity
certificate is issued by the CA and names the enclave, and that both signatures are valid. The tests
cover tampered documents and signatures from a foreign CA.

Interoperability with Fast DDS and Cyclone DDS loading these files has **not** been verified on this
machine (no ROS 2 installation). The formats follow sros2's output.
