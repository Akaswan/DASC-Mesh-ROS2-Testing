# DASC Mesh ROS 2 Testing

`mesh_ping` measures ROS 2 round-trip latency, loss, reordering and jitter between
two computers connected to different routers of the batman-adv mesh.

- `ping_node` publishes `mesh_ping/msg/Ping` on `/ping` with a sequence number,
  its own steady-clock send time and a filler payload.
- `pong_node` echoes every message back on `/pong`.
- RTT is computed on the ping machine only, so the clocks don't need to be synced.

Target: ROS 2 Foxy (Ubuntu 20.04), default RMW (Fast DDS). Foxy is EOL, so the code avoids Humble+ APIs.

## Build

### Docker (either machine)

The `ros:foxy-ros-base` image is multi-arch, so this works on the x86 laptop and a
64-bit OS on the Pi:

```bash
docker build -t mesh-ping .
```

Always run with host networking so DDS sees the real mesh interface:

```bash
docker run --rm -it --network host --ipc host -e ROS_DOMAIN_ID=42 mesh-ping <command>
```

### Native (Ubuntu 20.04 with Foxy installed, e.g. the Pi)

```bash
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Pre-flight checks

Run these in order between the two machines; each one depends on the previous.

1. `ping <other-ip>`, which checks L2/L3 across the mesh
2. `ros2 multicast receive` on one, `ros2 multicast send` on the other, which checks that DDS discovery traffic crosses the mesh
3. `ros2 run demo_nodes_cpp talker` / `listener` (needs the `ros:foxy` image or `ros-foxy-demo-nodes-cpp`)

Both machines must use the same `ROS_DOMAIN_ID` and must not set `ROS_LOCALHOST_ONLY=1`.
Allow UDP from the mesh subnet through any firewall.

## Run

Machine B (start first):

```bash
ros2 run mesh_ping pong_node
```

Machine A:

```bash
ros2 run mesh_ping ping_node --ros-args -p rate_hz:=50.0 -p payload_size:=1024
```

It prints one line per `report_period_s`, and a total summary on exit (Ctrl-C, or
automatically once `count` messages are sent and resolved).

### Parameters

| Node | Parameter | Default | Meaning |
|---|---|---|---|
| both | `reliable` | `false` | `true` = RELIABLE QoS, `false` = BEST_EFFORT. **Must match on both sides.** |
| both | `depth` | `10` | QoS history depth |
| ping | `rate_hz` | `10.0` | Send rate |
| ping | `payload_size` | `64` | Filler bytes per message |
| ping | `count` | `0` | Messages to send; `0` = until Ctrl-C |
| ping | `timeout_s` | `2.0` | No echo within this long → counted lost |
| ping | `report_period_s` | `1.0` | Interval between report lines |
| pong | `echo_payload` | `true` | `false` = strip payload on the reply, loading only the ping→pong direction |

### Output fields

- `lost` means no echo arrived within `timeout_s`. Loss % = lost / (recv + lost).
- `late` means an echo arrived after it was already counted lost, or a duplicate. Late messages are also counted in `lost`.
- `reord` means an echo arrived after one with a higher sequence number.
- `jitter` is the mean absolute difference between consecutive RTTs.
