# Mesh test guide

How to measure ROS 2 performance over the batman-adv mesh: what each test answers, how
to set it up physically, the exact commands, and how to read the results.

Contents:

1. [How the tooling fits together](#1-how-the-tooling-fits-together)
2. [One-time setup](#2-one-time-setup)
3. [Campaigns, runs, tags and the CSV files](#3-campaigns-runs-tags-and-the-csv-files)
4. [Rules for every field test](#4-rules-for-every-field-test)
5. The tests
   - [5.1 Distance falloff](#51-distance-falloff)
   - [5.2 Obstacles and line of sight](#52-obstacles-and-line-of-sight)
   - [5.3 Throughput limit](#53-throughput-limit)
   - [5.4 Hop count](#54-hop-count)
   - [5.5 Reliable vs best effort](#55-reliable-vs-best-effort)
   - [5.6 Walk test](#56-walk-test)
   - [5.7 Failover / reroute](#57-failover--reroute)
   - [5.8 Direction asymmetry](#58-direction-asymmetry)
   - [5.9 Discovery and background load](#59-discovery-and-background-load)
   - [5.10 Soak](#510-soak)
6. [Analysing results](#6-analysing-results)
7. [Troubleshooting](#7-troubleshooting)
8. [Command reference](#8-command-reference)

---

## 1. How the tooling fits together

```
 machine A (ping side)                                  machine B (pong side)
 ┌────────────────────────┐                             ┌──────────────────┐
 │ tools/mesh_test.py run │                             │    pong_node     │
 │   └─ ping_node ── /ping ──▶ router A ~~mesh~~ router B ──▶ echoes back │
 │        ◀────────── /pong ─────────────────────────────────── on /pong   │
 │   writes results/<campaign>/<run>/                   └──────────────────┘
 └────────────────────────┘
```

| Piece | Where it runs | What it does |
|---|---|---|
| `pong_node` | machine B | Echoes every `/ping` message back on `/pong`. Start it once and leave it running for the whole campaign. |
| `ping_node` | machine A, started by `mesh_test.py` | Waits until it has discovered pong, then sends numbered, timestamped messages and writes one CSV row per message outcome. |
| `tools/mesh_test.py` | machine A (and wherever you plot) | Runs `ping_node` with the right parameters, saves results, computes stats, and makes plots. |
| `load_node` + `tools/spawn_load.sh` | either machine | Adds extra DDS participants and/or background traffic (test 5.9). |

RTT is measured entirely on machine A (A's clock at send time vs. A's clock when the
echo arrives), so **the two machines' clocks never need to be synchronised**.

The distance that matters is between the two **routers**, since that's the mesh link
under test. Connect each computer to its router by a short Ethernet cable if you can.
If a computer joins its router over Wi‑Fi, that hop is part of every measurement too,
and it has to stay constant (same spot, right next to the router) for the results to
mean anything.

---

## 2. One-time setup

### 2.1 Build

Same as the [README](../README.md#build). Rebuild after pulling changes: the Docker image
(`docker build -t mesh-ping .`) or `colcon build`.

### 2.2 Python

`mesh_test.py run` only needs Python 3 (standard library). Plotting needs matplotlib,
usually on the laptop rather than the Pi:

```bash
python3 -m venv ~/.venvs/mesh && ~/.venvs/mesh/bin/pip install matplotlib
alias mesh-plot='~/.venvs/mesh/bin/python tools/mesh_test.py'
```

Results are plain folders, so you can record on the Pi and copy `results/` to the laptop
to plot (`rsync -a pi:DASC-Mesh-ROS2-Testing/results/ results/`).

### 2.3 Choose a ROS domain

Pick a `ROS_DOMAIN_ID` nobody else in the lab uses (0–101), and export it on **both**
machines in every terminal (put it in `~/.bashrc`):

```bash
export ROS_DOMAIN_ID=42
```

`mesh_test.py --docker` passes it into the container automatically. For `pong_node` in
Docker, pass it yourself with `-e ROS_DOMAIN_ID=42`.

### 2.4 Pre-flight

Do the [README pre-flight checks](../README.md#pre-flight-checks) once per session, before
the first run. If `ros2 multicast` fails, nothing below will work.

### 2.5 Signal strength capture (optional, strongly recommended)

Signal strength lets you plot loss against dBm, which transfers between environments far
better than distance does. `--signal-cmd` runs any shell command before and after each run,
stores its full output in `meta.json`, and pulls out the first `signal: -NN` it finds.

If the routers run OpenWrt with batman-adv:

```bash
# on router A: find the mesh radio interface and router B's MAC on it
batctl if            # e.g. "mesh0: active"
batctl n             # neighbour table: router B's MAC and last-seen
iw dev mesh0 station get <router-B-mac>    # look for "signal:" and "signal avg:"
```

Set up key-based SSH from machine A to router A so the command never prompts:

```bash
ssh-copy-id root@192.168.1.1        # router A's address
```

Then pass this to every run:

```bash
--signal-cmd "ssh root@192.168.1.1 'iw dev mesh0 station get <router-B-mac>; batctl o'"
```

Use `station get <mac>`, not `station dump`. A dump lists every neighbour, and the parser
takes the first `signal:` it sees, which might not be router B. `batctl o` adds the
originator table (TQ link quality, next hop) to the stored output, which is useful later
for checking hop counts.

If your routers aren't OpenWrt, any command that prints `signal: -NN` works. Without
one, the signal column is just empty.

### 2.6 Keep the session alive

If machine A is a Pi you SSH into **over the mesh**, the SSH session dies exactly when
the mesh does, and takes the test with it. Run tests inside `tmux` (or `screen`), or
from a keyboard and screen attached directly to machine A. For long runs, also stop the
laptop from sleeping (`systemd-inhibit --what=idle:sleep python3 tools/mesh_test.py ...`)
and keep it on mains power.

---

## 3. Campaigns, runs, tags and the CSV files

### 3.1 Vocabulary

- **Campaign**: one experiment, e.g. `hallway-2026-10-21`. Everything plotted together
  lives in one campaign folder. Use a new campaign when the *conditions* change (place,
  QoS, router firmware), and the same one when only the *variable you're testing* changes.
  Including the date in the name saves confusion later.
- **Run**: one execution of `ping_node` with one payload size and one rate. A
  `run` command with `--payload 64,1024 --rate 10,50 --repeat 2` makes 2 × 2 × 2 = 8 runs.
- **x variable**: what you're changing between positions. It's stored as `--distance`,
  as a `--tag`, or comes from the run's own settings (`rate`, `payload`, `load`, `signal`).
- **Series**: one line on a plot. The tool draws one per combination of whatever
  varies among campaign, payload, rate and QoS.

### 3.2 Tags

`--tag key=value` records anything about the run that you might want to group by or plot
against. Repeat it for several tags. Numbers (`walls=2`) are plotted on a numeric axis.
Text (`obstacle=door`) is plotted as categories, in the order you first recorded them.

Tags are free-form, but sticking to these names keeps campaigns comparable:

| Tag | Type | Meaning | Used in |
|---|---|---|---|
| `walls` | number | Walls between the two routers | 5.2 |
| `obstacle` | text | What's in the way: `none`, `door`, `glass`, `metal_shelf`, `person`… | 5.2 |
| `los` | 0/1 | 1 = clear line of sight between the routers | 5.1, 5.2 |
| `height_m` | number | Router height above the floor | 5.1 |
| `hops` | number | Mesh hops between router A and router B (check with `batctl tr`) | 5.4 |
| `spacing_m` | number | Distance between neighbouring routers in a multi-hop chain | 5.4 |
| `direction` | text | Which way the load flows, e.g. `A_to_B` / `B_to_A` | 5.8 |
| `echo` | 0/1 | 0 = pong was started with `echo_payload:=false` | 5.3, 5.8 |
| `participants` | number | Extra DDS participants spawned (total across both machines) | 5.9 |
| `bg_kbps` | number | Background traffic offered by `load_node` | 5.9 |
| `orig_interval_ms` | number | batman-adv originator interval (`batctl it`) | 5.7 |
| `txpower_dbm` | number | Router transmit power, if you change it | 5.1, 5.4 |
| `event` | text | Name of a scripted event: `R1_unplug`, `walk_out` | 5.6, 5.7 |
| `env` | text | `indoor` / `outdoor` / building name | any |

`--note "free text"` goes into `meta.json` and `summary.csv`. Use it for anything you'd
otherwise forget: weather, Wi‑Fi channel, "microwave on", "people walking through".

### 3.3 What gets written

```
results/
  hallway/                         ← campaign
    5m_1024B_10Hz_best_effort_20261021-142233/   ← one run
      raw.csv
      meta.json
      ping.log
    summary.csv                    ← written by `plot`
    hallway_vs_distance.png        ← written by `plot`
```

#### `raw.csv`: one row per message outcome (written by `ping_node`)

| Column | Meaning |
|---|---|
| `seq` | Sequence number, from 0 |
| `send_ns` | Machine A's monotonic clock at send (nanoseconds; only differences are meaningful) |
| `rtt_ms` | Round-trip time; empty for `lost` |
| `status` | `ok` = echoed in order. `reordered` = echoed after a higher seq had already come back. `lost` = no echo within `timeout_s`. `late` = an echo for a seq already counted lost (or a duplicate). |

Rows are written as outcomes happen, so they're in **arrival order**, not seq order.
Messages still in flight when a run is interrupted have no row.

#### `meta.json`: everything about one run

`campaign`, `run_id`, `distance_m`, `tags`, `note`, `qos`, `params` (exactly what
`ping_node` got), `host`, `ros_domain_id`, `started` / `finished` (wall clock),
`signal_before` / `signal_after` (raw command output + parsed `dbm`), `marks` (from
`--marks`), `completed` (false if it hit the deadline or you aborted a fixed-length run),
and `stats` (below).

#### `summary.csv`: one row per run (written by `plot`)

Identity columns: `campaign, run_id, started, distance_m, <every tag>, payload_size,
rate_hz, qos, signal_dbm, completed`. Then the stats:

| Column | Meaning | Why you care |
|---|---|---|
| `sent` | Messages sent | |
| `received` | Echoes that arrived in time (`ok` + `reordered`) | |
| `lost` | No echo within `timeout_s` | |
| `loss_pct` | `lost / (received + lost)` × 100 | The headline number. In-flight messages are excluded. |
| `max_loss_burst` | Longest run of consecutive lost seqs | 2 % loss as scattered singles is harmless; 2 % as one 3-second hole stops a robot. |
| `max_gap_s` | Longest time between two consecutive *delivered* messages (by send time) | The outage a subscriber actually experiences. The key failover metric. |
| `late` | Echoes that arrived after being counted lost | Many late → `timeout_s` is too short for this link (common with reliable QoS). |
| `reordered` | Echoes that overtook each other | Usually rare. Rises during reroutes. |
| `rtt_min_ms` … `rtt_max_ms` | min, mean, p50, p95, p99, max RTT | p50 = typical latency; p99 = what control loops must tolerate. |
| `jitter_ms` | Mean \|RTT − previous RTT\| | Variability; matters for time-sensitive control. |
| `offered_kbps` | `rate × payload × 8 / 1000`, one direction | What you asked the link to carry. |
| `goodput_kbps` | Payload actually delivered per second | Equal to offered until the link saturates. |
| `discovery_ms` | Time from `ping_node` starting until it matched pong in both directions | How long a new node takes to "see" the other side. |
| `note` | Your `--note` | |

You can open `summary.csv` directly in a spreadsheet, or use pandas
(`pd.read_csv('results/hallway/summary.csv')`).

---

## 4. Rules for every field test

These matter more than any tool feature:

1. **Change one thing at a time.** Keep router height, antenna orientation, laptop
   position relative to its router, Wi‑Fi channel and time of day as constant as you can.
   Record anything that does change as a tag or note.
2. **Mount the routers at a fixed height,** about 1 m on a stool, box or tripod, not on
   the floor. Floor-level radios lose a lot of range, and inconsistently. Point antennas
   the same way every time (vertical is the usual default).
3. **Measure distance properly.** Use a tape measure or laser rangefinder between the
   router antennas, and mark the positions on the floor with tape so you can come back to them.
4. **Space positions roughly logarithmically:** e.g. 1, 2, 5, 10, 15, 20, 30, 40, 50, 75,
   100 m. Radio signal falls off with the log of distance, so near positions change fast
   and far ones slowly. Once you see loss start, add extra positions around that point.
5. **Repeat.** Use `--repeat 2` or more at each position, and if time allows, do an outward
   pass then a return pass (same campaign, same positions). The plot shows each repeat as
   a faint dot and draws the line through their mean, so the scatter shows you how
   trustworthy each point is.
6. **Stay out of the path.** Your body absorbs 2.4/5 GHz signal. Stand behind the router
   that's sending, not between the two.
7. **Note the RF environment.** Other Wi‑Fi networks on the same channel are the most
   common hidden variable. Run `iw dev <iface> scan | grep -E "SSID|freq|signal"` on a
   router (or use a phone Wi‑Fi analyser app) and put the busy channels in `--note`.
8. **Keep SSH and other traffic off the mesh during runs.** See §2.6.
9. **Sanity-check at 1 m first.** Every campaign should start with a near run that shows
   ~0 % loss and low single-digit-millisecond p50. If it doesn't, fix that before going further.

Default run settings unless a test says otherwise: `best_effort` QoS (most sensor
topics use it), 30 s per run, and both a small payload (64 B, like a command or odometry)
and a medium one (1024 B, like a small sensor message).

---

## 5. The tests

Each test lists: what it answers, what you need, the physical procedure, the commands,
and how to read the result. In all the commands, `T=python3 tools/mesh_test.py`. Add
`--docker mesh-ping` if machine A runs ROS in Docker, and your `--signal-cmd` if you set
one up.

On machine B, unless the test says otherwise:

```bash
ros2 run mesh_ping pong_node
# or
docker run --rm -it --network host --ipc host -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID mesh-ping \
    ros2 run mesh_ping pong_node
```

### 5.1 Distance falloff

**Answers:** how loss, latency and jitter change as the two routers move apart, and the
distance where the link stops being usable.

**You need:** a long open space (corridor, field, car park), tape measure, floor tape,
two stools or tripods.

**Procedure:**

1. Put router B (with machine B) at a fixed end and start `pong_node`.
2. Put router A on its stand 1 m away, with machine A next to it.
3. Run the first position (the tool asks for confirmation before starting):

   ```bash
   $T run --campaign hallway-1021 --distance 1 --tag los=1 --tag height_m=1 \
       --payload 64,1024 --rate 20 --repeat 2 --note "corridor B2, ch 36"
   ```

   That's 4 runs × 30 s ≈ 2 min per position. Each run prints a one-line summary when it
   finishes.
4. Move router A to the next mark (2 m) and repeat the command with `--distance 2`.
   Keep going until loss goes over 50 % or discovery times out ("waiting for
   pong_node..." for 30 s means you're out of range).
5. Optional: walk back in, re-running a few of the same positions to check for drift.
6. Plot:

   ```bash
   $T list --campaign hallway-1021
   $T plot --campaign hallway-1021
   $T plot --campaign hallway-1021 --x signal --panels loss,p99   # if you captured signal
   $T plot --campaign hallway-1021 --panels loss,burst,gap,signal
   ```

**Reading it:** Expect a flat region (≈0 % loss, stable RTT), then a "knee" where p99 RTT
and jitter climb first, then loss rises steeply. The knee is your usable range,
**not** the point where loss is total. Note where it is for each payload: 1024 B usually
fails earlier than 64 B, because anything over ~1400 B is split across several radio
frames (IP fragmentation), and losing any one of them loses the whole message.

**Variations:** repeat with `--tag height_m=0.2` and `height_m=2`, or indoors vs outdoors
(new campaign each, then `plot --campaign a,b` to overlay them).

### 5.2 Obstacles and line of sight

**Answers:** how much specific obstacles cost: walls, doors, metal shelving, people.

**Procedure:** pick **one fixed distance** where the open-air link is clearly good
(from 5.1, somewhere well inside the knee). Then change only what's between the routers:

```bash
$T run --campaign obstacles-1021 --distance 8 --tag obstacle=none   --payload 64,1024 --repeat 3
$T run --campaign obstacles-1021 --distance 8 --tag obstacle=door   --payload 64,1024 --repeat 3
$T run --campaign obstacles-1021 --distance 8 --tag obstacle=wall   --payload 64,1024 --repeat 3
$T run --campaign obstacles-1021 --distance 8 --tag obstacle=shelf  --payload 64,1024 --repeat 3
$T plot --campaign obstacles-1021 --x obstacle --panels loss,p50,p99,signal
```

For counting walls (e.g. going room-by-room down a building), use a number so it plots
as a trend:

```bash
$T run --campaign walls-1021 --distance 12 --tag walls=0 ...
$T run --campaign walls-1021 --distance 15 --tag walls=1 ...
$T plot --campaign walls-1021 --x walls
```

When distance changes between wall counts (it usually has to), plotting against
`--x signal` separates the two effects better than plotting against walls.

**Reading it:** text categories are drawn as dots (no lines, since categories have no
order). Compare each obstacle's loss and dBm against `none`. A drop of 10 dB or more
from a single obstacle is significant.

### 5.3 Throughput limit

**Answers:** how much data ROS can actually push across the mesh before loss and
latency blow up. It tells you whether camera, point cloud or map topics will fit.

**Procedure:** two positions: one near (≈2 m) and one around the falloff knee from 5.1.
At each, sweep payload size × rate:

```bash
$T run --campaign tput-near-1021 --distance 2 \
    --payload 64,1024,8192,32768,60000 --rate 10,50,100,200,500 --duration 15 --pause 2
```

That's 25 runs × ~15 s ≈ 8 min. `--pause 2` lets queues drain between runs. Then:

```bash
$T plot --campaign tput-near-1021 --x load --logx --panels loss,goodput,p50,p99
```

**Reading it:** the x axis is offered load (kbit/s, one way). Each line is one payload size.
Goodput tracks the diagonal until the link saturates, then flattens. That flat level
is your capacity. Loss and p99 climb at the same point. The capacity for large payloads
is usually much higher than for small ones at the same message rate, because per-message
overhead dominates small messages. For the robot, use the curve that matches the real
message size of the topic you care about.

The echo doubles the traffic: the mesh carries the load in both directions. To load only
A→B, start pong with `-p echo_payload:=false` and tag runs with `--tag echo=0`. Goodput
then means one-way delivered payload.

**Limits:** `ping_node`'s timer is reliable up to a few hundred Hz. Above ~1000 Hz,
measure capacity with bigger payloads instead. Payloads much over 60 KB also exercise DDS
fragmentation, which is a separate test.

If a laptop and Pi are involved, watch `top` on the Pi during the highest-load runs.
A CPU pinned at 100 % means you've measured the Pi, not the mesh.

### 5.4 Hop count

**Answers:** how much each additional mesh hop costs in latency, loss and throughput.
On single-radio meshes, throughput roughly halves per hop, because each relay has to
receive and retransmit on the same channel.

**You need:** at least one more router as a relay (R1, R2…).

**Forcing multiple hops.** batman-adv always picks the best path, so A and B have to be
unable to hear each other directly. Options, from most to least realistic:

1. **Distance or walls:** place A and B beyond direct range (from 5.1/5.2), with R1
   halfway between.
2. **Lower transmit power** so range shrinks to fit indoors:
   `iw dev mesh0 set txpower fixed 500` (5 dBm, in mBm) on every router, then tag runs
   `--tag txpower_dbm=5`. Restore it afterwards (`iw dev mesh0 set txpower auto`).
3. **Block the direct link** on routers A and B by dropping the other end's frames on
   the mesh radio. This makes 2 hops possible in one room, but the RF environment isn't
   realistic. How you do it depends on the firmware:
   - If the mesh interface is 802.11s (`iw dev` shows `type mesh point`):
     `iw dev mesh0 station set <other-end-mac> plink_action block` (undo with `open`).
   - Otherwise drop the frames at ingress with `tc` (OpenWrt: install `tc` and
     `kmod-sched-flower`):
     `tc qdisc add dev mesh0 clsact; tc filter add dev mesh0 ingress flower src_mac <other-end-mac> action drop`
     (undo with `tc qdisc del dev mesh0 clsact`).

   `ebtables`/`iptables` won't work here: batman-adv's own frames on `mesh0` never pass
   through a bridge or the IP stack.

**Always verify the hop count** before each run, on router A:

```bash
batctl tr <router-B-mac>     # traceroute: one line per hop
batctl o                     # originator table: "Nexthop" for router B
```

**Procedure:**

```bash
# 1 hop (direct)
$T run --campaign hops-1021 --tag hops=1 --tag spacing_m=10 --payload 64,1024,8192 --rate 20,200 --repeat 2
# 2 hops via R1
$T run --campaign hops-1021 --tag hops=2 --tag spacing_m=10 --payload 64,1024,8192 --rate 20,200 --repeat 2
# 3 hops via R1, R2 ...
$T plot --campaign hops-1021 --x hops --panels loss,p50,p99,goodput
```

Put `batctl tr` in your `--signal-cmd` so the route used is stored in each run's `meta.json`:
`--signal-cmd "ssh root@<router-A> 'batctl tr <router-B-mac>'"`. If a run's stored
traceroute shows a different number of hops than its tag, that run is mislabelled.

**Reading it:** p50 should rise roughly linearly with hops. Goodput at high load should
fall to roughly ½, ⅓ … of the 1-hop value. Loss that grows faster than that points to
a weak link in the chain, so check the per-hop signal.

### 5.5 Reliable vs best effort

**Answers:** what RELIABLE QoS buys and costs on a lossy link. Reliable hides loss
by retransmitting, which shows up as latency spikes and stalls.

**Procedure:** run the same distance sweep (5.1) twice, as two campaigns. QoS has to
match on both ends, so restart pong between them:

```bash
# pass 1: best effort (pong started normally)
$T run --campaign qos-be-1021  --distance 10 --qos best_effort --payload 64,1024 --repeat 2
# pass 2: restart pong with -p reliable:=true, then
$T run --campaign qos-rel-1021 --distance 10 --qos reliable --timeout 5 --payload 64,1024 --repeat 2
```

`--timeout 5` (instead of 2) for reliable: retransmitted messages can take seconds
to arrive, and with a 2 s timeout they'd be counted lost and then show up as `late`.
Do each position in both modes before moving (switching pong each time), or do a full
pass per mode. The second takes fewer restarts but is more exposed to the environment
changing in between.

```bash
$T plot --campaign qos-be-1021,qos-rel-1021 --panels loss,p50,p99,gap
```

**Reading it:** at short range the two are identical. Past the knee, reliable shows lower
loss but much higher p99 and `max_gap_s`. Reliable also eventually collapses
abruptly when it can't keep up. For control topics a fresh message matters more than
an old one, so best effort's loss is often the better trade. Use this plot to justify the
QoS choices per topic.

**Note:** `depth` (history depth) matters with reliable. A small depth with a fast
publisher drops messages from the writer's queue under load. Try `--depth 100` as a
variation.

### 5.6 Walk test

**Answers:** where exactly the link degrades and breaks as one end moves continuously, and
what happens on the way back (does it recover at the same distance?).

**You need:** machine A and router A both mobile: battery router (or a power bank), the
laptop on a cart or carried, floor marks every 5 m.

**Procedure:**

1. Router B + pong fixed. Router A + machine A at the 0 m mark.
2. Start an open-ended run with marks:

   ```bash
   $T run --campaign walk-1021 --tag event=walk_out_back --rate 20 --duration 0 --marks -q \
       --note "walk speed ~0.5 m/s, pause 10 s per mark"
   ```

   `--duration 0` runs until Ctrl-C. `--marks` means each time you press **Enter** the
   moment is recorded. Type a label first (e.g. `15`) to name it. `-q` hides the
   per-second lines so your typing stays visible.
3. Walk slowly. At each floor mark, stop, type the distance (`5`, Enter), wait ~10 s,
   continue. Walk out until the link is clearly dead (100 % loss for ~20 s), type `turn`,
   then walk back the same way, marking again.
4. Back at 0 m, wait 10 s and press **Ctrl-C**. The run is saved as complete.
5. Plot:

   ```bash
   $T timeline --campaign walk-1021
   ```

**Reading it:** the timeline shows loss % and median/p99 RTT per time bin, with a dotted
line and label at each mark. Read off the mark where loss starts and the one where it
reaches 100 %. If the link comes back at a *shorter* distance on the return than where it
dropped going out, that's hysteresis (route and discovery recovery time). The
`max_gap_s` in the title is the worst outage.

`timeline` picks the latest run by default. Use `--run <run_id prefix>` for an older one,
`--bin 1` to force 1 s bins, and `--logy` when RTT spikes squash everything else.

### 5.7 Failover / reroute

**Answers:** when a relay in the active path disappears, how long the robot loses
connectivity before batman-adv reroutes. This is often the most operationally
important number.

**You need:** a topology with two possible paths, e.g. A and B out of direct range, with
two relays R1 and R2 each in range of both:

```
        R1
      /    \
    A        B        (A and B can't hear each other)
      \    /
        R2
```

**Procedure:**

1. Confirm both relays work: `batctl o` on router A should list B, with a next hop of R1
   or R2. Note which one it is (say R1).
2. Start:

   ```bash
   $T run --campaign failover-1021 --tag event=R1_unplug --tag orig_interval_ms=1000 \
       --rate 50 --duration 0 --marks -q
   ```

   50 Hz gives 20 ms resolution on the outage length.
3. Wait 30 s for a clean baseline. Type `R1 off`, press Enter, and **at the same moment**
   pull the power on R1 (the active next hop).
4. Wait until traffic recovers, then 60 s more. Type `R1 on` + Enter, plug R1 back in.
5. Wait 60 s (watch whether the route flaps back to R1) and Ctrl-C.
6. Repeat 3–5 times. The outage length varies from run to run.

```bash
$T timeline --campaign failover-1021 --bin 0.25
$T list --campaign failover-1021 --x event      # the gap s column = outage per trial
$T plot --campaign failover-1021 --x orig_interval_ms --panels gap,burst
```

**Reading it:** `max_gap_s` is the outage. With batman-adv defaults it's typically several
seconds, because a route only changes once the originator messages (OGMs) through R1
stop arriving. To measure the effect of tuning, change the originator interval on **all**
routers (`batctl it 500`) and repeat, tagging `orig_interval_ms=500`. Shorter intervals
fail over faster but add background traffic. Run 5.9's background load test alongside
if that matters to you.

**Variations:** instead of unplugging, walk R1 out of range (gradual failure, which batman
handles differently from a sudden one), or block R1 with the frame-drop method from 5.4 option 3, which
can be switched on at a precise moment and repeated exactly.

### 5.8 Direction asymmetry

**Answers:** whether the link is worse one way than the other. Different hardware,
antennas or local interference on one end make this common.

The limitation to keep in mind: ping/pong measures the **round trip**. A lost message
could have been lost on either leg, and RTT is the sum of both. To isolate a direction,
make that direction carry nearly all the traffic and measure it from both ends.

**Procedure:**

1. Pong on B with a stripped echo:
   `ros2 run mesh_ping pong_node --ros-args -p echo_payload:=false`
2. On A, load A→B:

   ```bash
   $T run --campaign asym-1021 --distance 15 --tag direction=A_to_B --tag echo=0 \
       --payload 8192 --rate 50,200 --repeat 3
   ```
3. Swap roles: pong (with `echo_payload:=false`) on **A**, and run `mesh_test.py` on **B**
   with `--tag direction=B_to_A`. Use the same campaign name and copy B's `results/` folder
   over to A's afterwards (`rsync`).
4. `$T plot --campaign asym-1021 --x direction --panels loss,p99,goodput`

**Reading it:** at a distance near the knee, a gap of more than a few percentage points
in loss between the two directions is real asymmetry. Check `signal_before` from both
routers (station dump on each side reports the signal *it* receives).

### 5.9 Discovery and background load

**Answers:** two related things.

- **(a) Discovery:** how long a new node takes to find its peer as the number of ROS
  participants grows. DDS discovery uses multicast, which Wi‑Fi sends at the slowest basic
  rate, so on a mesh it gets expensive quickly.
- **(b) Background load:** how much other ROS traffic on the mesh degrades the latency
  and loss of your ping topic, i.e. how topics interfere with each other.

`load_node` creates `topics` publishers and/or subscribers on `load_0 … load_N`.
`spawn_load.sh N` starts N `load_node` processes, and each process is one DDS participant
(in Foxy, one per process). Ctrl-C stops them all.

```bash
tools/spawn_load.sh N [-p topics:=K] [-p rate_hz:=R] [-p payload_size:=B]
                      [-p publish:=true|false] [-p subscribe:=true|false]
# in Docker:
docker run --rm -it --network host --ipc host -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -v "$PWD/tools:/tools" mesh-ping /tools/spawn_load.sh N ...
```

**(a) Procedure, discovery vs participants.** Keep the routers at a fixed, good distance.
For each N, start the load on **both** machines (N/2 each, so discovery traffic crosses
the mesh), wait ~20 s for it to settle, then do several short runs. Each run is a fresh
`ping_node`, so each one measures one discovery:

```bash
# machine A and B: tools/spawn_load.sh 5 -p topics:=3   (N = 10 total)
$T run --campaign disc-1021 --distance 5 --tag participants=10 --duration 5 --repeat 10 -y -q
# Ctrl-C the spawners, go to the next N: 0, 10, 20, 40, 80
$T plot --campaign disc-1021 --x participants --panels discovery,loss,p50
```

`--repeat 10` matters: discovery time is noisy, and the faint dots show the spread.
The participant count is exact. If you also vary `topics`, add it as a tag (`--tag endpoints=…`).

**(b) Procedure, background traffic.** One-way background stream A→B across the mesh,
at increasing rates:

```bash
# machine A (publishers only):
tools/spawn_load.sh 1 -p topics:=10 -p rate_hz:=50 -p payload_size:=1024 -p subscribe:=false
# machine B (subscribers only), so the traffic actually crosses the mesh:
tools/spawn_load.sh 1 -p topics:=10 -p publish:=false
```

`load_node` logs its offered kbit/s at start, and B's process logs how much it actually
received every 5 s. Then:

```bash
$T run --campaign bgload-1021 --distance 5 --tag bg_kbps=4096 --payload 64 --rate 20 --repeat 2
```

Repeat for bg_kbps = 0, 1000, 4000, 8000…. To change it, alter `rate_hz` or `payload_size`.
10 topics × 50 Hz × 1024 B × 8 = 4096 kbit/s.

```bash
$T plot --campaign bgload-1021 --x bg_kbps --panels loss,p50,p99,jitter
```

**Reading it:** (a) discovery time growing superlinearly with participants is a sign
you'll want a discovery server or fewer processes on the real robot. (b) Once ping latency
rises sharply, the background traffic is filling the channel. Compare that bg_kbps with
the capacity from 5.3.

### 5.10 Soak

**Answers:** what happens over hours: occasional dropouts, periodic latency spikes,
memory or thermal problems on the Pi.

**Procedure:**

```bash
tmux new -s soak
$T run --campaign soak-1021 --distance 10 --rate 10 --duration 0 -q --note "overnight, lab door closed"
# detach (Ctrl-b d); come back hours later: tmux attach -t soak; Ctrl-C
$T timeline --campaign soak-1021
$T timeline --campaign soak-1021 --bin 60 --logy
```

10 Hz for 8 h is about 290,000 rows (~10 MB). That's fine. Higher rates also work.

**Reading it:** look for

- **Periodic spikes** (e.g. every 60 s or 120 s): background scans, batman-adv
  housekeeping, DHCP renewals, or another device's schedule. Measure the period with
  `--bin 1` and zoom in by opening the PNG.
- **Isolated outages:** check `max_gap_s` and line up the times with anything else
  that happened. The timeline x axis is seconds since the first message, which is
  `started` in `meta.json` (wall clock) plus the run's `discovery_ms`.
- **Slow drift** in median RTT: thermal throttling on the Pi (`vcgencmd measure_temp`)
  or a filling queue.

---

## 6. Analysing results

| Command | What it does |
|---|---|
| `$T list --campaign C [--x VAR]` | One line per run: x value, config, sent, loss %, burst, gap, p50, p99, jitter, goodput, discovery, dBm. Incomplete runs are flagged. |
| `$T plot --campaign C [--x VAR] [--panels ...] [--logx]` | Writes `summary.csv` and `<C>_vs_<VAR>.png` in the campaign folder. |
| `$T plot --campaign C1,C2,…` | Overlays campaigns (each becomes its own series). Output goes to `results/C1+C2/`. |
| `$T timeline --campaign C [--run PREFIX] [--bin S] [--logy]` | Writes `timeline.png` in the run's folder. |

`--x` options: `distance`, `rate`, `payload`, `load` (offered kbit/s), `signal` (dBm), or
any tag name. Runs without a value for the chosen x are left out.

`--panels` options (default `loss,p50,p99,jitter`): `loss`, `p50`, `p99`, `jitter`,
`burst`, `gap`, `goodput`, `discovery`, `signal`.

Series are made automatically from whatever varies among campaign, payload, rate and QoS.
More than 8 series is refused, so split the campaign or plot subsets. Repeated runs at the
same x are averaged into the line, and the individual runs are drawn as faint dots.

For anything the built-in plots don't cover, load `summary.csv` (per run) or the
`raw.csv` files (per message) into a spreadsheet or pandas. Every number the tool reports
can be recomputed from `raw.csv`.

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Stuck on `waiting for pong_node...`, run ends incomplete with sent=0 | Pong not running, different `ROS_DOMAIN_ID`, QoS mismatch (`--qos` vs pong's `reliable`), `ROS_LOCALHOST_ONLY=1` set, or out of range. Re-run the pre-flight checks. |
| Loss at 1 m | Laptop↔router link (Wi‑Fi?) is the problem, another network on the channel, or the machine is CPU-bound. Check `ping` loss between the machines first. |
| Lots of `late` | `timeout_s` is shorter than the real RTT spikes. Raise `--timeout` (always with reliable QoS). |
| `no raw.csv written` | `ping_node` crashed or couldn't open the file. See `ping.log` in the run folder. In Docker mode, check the image is rebuilt (`docker build -t mesh-ping .`). |
| `raw.csv` owned by root | Expected in Docker mode. You can still read it and delete it. |
| Plot refuses: "too many series" | More than 8 payload/rate/QoS/campaign combinations. Plot fewer campaigns, or put fewer payloads/rates in each. |
| `spawn_load.sh` processes die at start | Rebuild the image. Older versions had a log-directory race. |
| Signal column empty | `--signal-cmd` output has no `signal:` line. Run the command by hand, and check SSH works without a password. |
| `timeline` marks are missing | Marks only come from runs made with `--marks`. Typing without pressing Enter doesn't record one. |

---

## 8. Command reference

```
mesh_test.py run --campaign NAME (--distance M | --tag K=V ...) [options]
  --tag K=V            extra variable, repeatable (§3.2)
  --note TEXT          free text saved with the run
  --payload 64,1024    payload sizes in bytes (comma list)          default 64
  --rate 10,50         send rates in Hz (comma list)                default 10
  --qos Q              best_effort | reliable (must match pong)      default best_effort
  --duration S         seconds per run; 0 = until Ctrl-C            default 30
  --repeat N           repeat every payload×rate combination        default 1
  --pause S            wait between runs                            default 0
  --timeout S          echo deadline before a message counts lost   default 2
  --depth N            QoS history depth                            default 10
  --discovery-timeout  seconds to wait for pong before giving up    default 30
  --docker IMAGE       run ping_node in this image
  --signal-cmd CMD     command whose output has "signal: -NN"
  --marks              Enter during the run records a labelled mark
  -q / --quiet         hide per-second output
  -y / --yes           don't ask for confirmation before starting

mesh_test.py list     --campaign C[,C2] [--x VAR]
mesh_test.py plot     --campaign C[,C2] [--x VAR] [--panels a,b,..] [--logx]
mesh_test.py timeline --campaign C [--run PREFIX] [--bin S] [--logy]

ping_node  params: rate_hz, payload_size, count, timeout_s, report_period_s, reliable, depth, csv_path
pong_node  params: reliable, depth, echo_payload
load_node  params: topics, rate_hz, payload_size, publish, subscribe, reliable, depth
spawn_load.sh N [load_node -p params...]
```
