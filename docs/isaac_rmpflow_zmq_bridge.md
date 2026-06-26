# Isaac RMPFlow ZMQ Bridge

Status: implemented scaffold. The ROS bridge and Isaac/Lula ZMQ server exist,
the generated primitive URDF converts to USD, and the RMP server accepts the
bridge request schema. The generated collision spheres, initial articulation,
and RMPFlow gains still require visual/operator validation before hardware use.

## Recommendation

Run Isaac RMPFlow and FACTR ROS as two separate Python processes:

```text
factr_conda_env process                     env_isaaclab process
----------------------                     ---------------------
ROS bridge node             ZMQ stream      Isaac RMPFlow server
- rclpy, ROS 2 topics  ----------------->  - Isaac Lab / Isaac Sim
- FACTR topic contract   joint requests     - Lula / RMPFlow world
- freshness watchdogs    <----------------  - collision scene
- safe_ur_pos publisher    joint response   - no ROS dependency
```

The FACTR process remains the only robot-facing process. It owns ROS, RTDE, UR
startup safety checks, leader matching, and the final `servoJ` command path. The
Isaac process owns Isaac Lab, USD/articulation setup, RMPFlow, and collision
world state. ZMQ is only the boundary between those two environments.

Run only one active safe-target producer at a time. The old OpenPI-YAM
collision monitor and the Isaac/Lula stream bridge publish the same
`/factr_teleop/<side>/safe_ur_pos` topic, so do not run both while teleop is
using `collision_safety:=true`.

## High-Rate Streaming Path

The bridge subscribes to the existing FACTR streams:

```text
/ur/<side>/obs_ur_state
/factr_teleop/<side>/desired_ur_pos
```

It should publish the only command-bearing Isaac output:

```text
/factr_teleop/<side>/safe_ur_pos
```

The bridge should keep ROS topic data in real UR joint coordinates. Any Isaac
joint-order, wrist-offset, frame, or base-pose conversion belongs inside the
bridge/server boundary, not in the teleop loop.

The current RMPFlow runtime is Isaac/Lula only. The previous experimental
non-Isaac RMP mode was removed from `ur7e_collision_monitor.py`; that node now
keeps only the older position/QP-style safety path with `velocity` and
`posture` modes.

```text
FACTR ROS topics        latest input stream        Isaac/Lula process
obs_q, desired_q  ----------------------------->  fixed-rate RMPFlow loop
safe_q topic      <-----------------------------  latest safe target stream
UR servoJ tracks safe_q
```

Run the streaming server from `env_isaaclab`:

```bash
cd /home/srianumakonda/FACTR_Teleop

bash scripts/isaac_rmpflow/run_lula_stream_server.sh \
  --mode rmp \
  --input-endpoint tcp://127.0.0.1:5558 \
  --output-endpoint tcp://127.0.0.1:5559 \
  --loop-hz 500.0 \
  --policy-sides right \
  --dynamic-other-arm-obstacles \
  --require-other-arm-state
```

Run the ROS streaming bridge from `factr_teleop`:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/isaac_rmpflow_stream_bridge.py \
  active_sides:=right \
  input_endpoint:=tcp://127.0.0.1:5558 \
  output_endpoint:=tcp://127.0.0.1:5559 \
  publish_hz:=500.0 \
  max_joint_step_rad:=0.05 \
  max_safe_target_distance_rad:=0.05 \
  publish_safe_targets:=false
```

The streaming server publishes status once per second. A healthy 500 Hz loop
should report roughly `published: 500` per second and `last_ok: true` while
fresh ROS inputs are arriving. If inputs stop, it switches to `ok: false` with
an input-age reason, and the ROS bridge should publish no new safe target.

For shadow validation, keep `publish_safe_targets:=false` and inspect:

```bash
ros2 topic echo /factr_teleop/isaac_rmpflow_stream/status
ros2 topic echo /factr_teleop/isaac_rmpflow_stream/reason
ros2 topic echo /factr_teleop/right/isaac_stream_safe_error
```

Or collect a 30 second shadow-health summary:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

python scripts/isaac_rmpflow/collect_stream_shadow_diagnostics.py \
  --active-sides right \
  --duration-s 30 \
  --output-json /tmp/isaac_rmpflow_shadow_right.json
```

Go only when the JSON summary has `shadow_healthy: true`, zero
`safe_ur_pos_counts` for every active side, fresh observed/desired topics, a
fresh other-arm observed topic when using dynamic other-arm obstacles,
`controller_hz` present and near the expected loop rate, low `input_age_ms`, and
empty `missing_required_topics` and `stale_required_topics`.

`publish_safe_targets:=false` is shadow only: it must not publish
`/factr_teleop/<side>/safe_ur_pos`. During shadow mode, run FACTR teleop without
`collision_safety:=true` so normal FACTR motion continues while the RMPFlow
stream is observed. The first active command path is relaunching the bridge with
`publish_safe_targets:=true` and the same conservative hardware bring-up
limits:

```bash
ros2 launch launch/isaac_rmpflow_stream_bridge.py \
  active_sides:=right \
  input_endpoint:=tcp://127.0.0.1:5558 \
  output_endpoint:=tcp://127.0.0.1:5559 \
  publish_hz:=500.0 \
  max_joint_step_rad:=0.05 \
  max_safe_target_distance_rad:=0.05 \
  publish_safe_targets:=true
```

Use `collision_safety:=true` only after relaunching the bridge with
`publish_safe_targets:=true`. For first hardware bring-up, also pass
`safe_target_timeout:=0.10` to the UR7e teleop launch so stale safe targets are
dropped quickly.

`--dynamic-other-arm-obstacles` initializes the left arm as Lula sphere
obstacles at the configured left initial pose and updates those obstacles when
fresh `/ur/left/obs_ur_state` messages arrive. For real moving bimanual
validation, run a left-state publisher and keep `--require-other-arm-state` in
the stream-server command so missing left observations fail closed. Omitting
`--require-other-arm-state` is only for static-left/static-other-arm tests.

## ZMQ Pattern

Use the latest-only streaming PUB/SUB path for RMPFlow bring-up. Keep ROS-side
stale-output timeouts short enough that a stuck Isaac process cannot feed stale
targets to the robot.

Recommended streaming defaults:

```text
input_endpoint: tcp://127.0.0.1:5558
output_endpoint: tcp://127.0.0.1:5559
loop_hz: 500
publish_hz: 500
state_timeout_s: 0.10
desired_timeout_s: 0.10
max_response_age_s: 0.10
max_joint_step_rad: 0.05
max_safe_target_distance_rad: 0.05
safe_target_timeout: 0.10
```

The older request/reply bridge was removed; do not use `tcp://127.0.0.1:5557`
for the current RMPFlow path.

## Request Schema

Use JSON for the first pass. Switch to msgpack later if serialization cost
matters. All joint arrays are six-element UR arm vectors in radians unless a
field says otherwise.

```json
{
  "schema": "factr.isaac_rmpflow.request.v1",
  "sequence": 42,
  "stamp": 1718900000.123,
  "active_sides": ["right"],
  "arms": {
    "right": {
      "q_current": [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0],
      "q_desired": [-1.56, -1.57, -1.58, -1.56, 1.57, 0.02],
      "state_age_s": 0.006,
      "desired_age_s": 0.004
    }
  },
  "limits": {
    "max_joint_step_rad": 0.05
  }
}
```

Bridge-side validation before send:

- active side is known: `left`, `right`, or both
- each active arm has fresh current and desired messages
- arrays have exactly six finite numbers
- requested step is bounded or clipped before Isaac sees it
- sequence number is monotonic

## Response Schema

```json
{
  "schema": "factr.isaac_rmpflow.response.v1",
  "sequence": 42,
  "stamp": 1718900000.134,
  "ok": true,
  "mode": "filtered",
  "arms": {
    "right": {
      "q_safe": [-1.56, -1.57, -1.58, -1.56, 1.57, 0.02],
      "limit_margin_min": 0.38,
      "distance_margin_min_m": 0.11
    }
  },
  "reason": "rmpflow_clear"
}
```

Allowed `mode` values:

```text
pass_through: Isaac accepted the desired target unchanged
filtered: Isaac returned a nearby safer target
hold: do not publish a new safe target; keep the robot on FACTR's timeout path
stop: operator intervention or launch shutdown is required
```

Bridge-side validation before publish:

- response schema and sequence match the latest request
- response arrives before `isaac_response_timeout_s`
- `ok` is true and `mode` is `pass_through` or `filtered`
- each `q_safe` vector has exactly six finite numbers
- `q_safe` is close enough to `q_current` and `q_desired`
- Isaac-side transforms were applied exactly once

If any check fails, publish no new `/factr_teleop/<side>/safe_ur_pos`.

## Safety Behavior

The bridge must fail silent on command output. On stale input, missing Isaac
responses, invalid arrays, mismatched sequences, transform errors, or an Isaac
`hold` / `stop` response, it should skip publishing `safe_ur_pos` and report a
diagnostic reason.

FACTR's UR7e node should remain responsible for deciding what stale
`safe_ur_pos` means during real execution. The bridge timeout should be shorter
than FACTR's safe-target timeout so the robot never chases old Isaac output.

Recommended diagnostics:

```text
/factr_teleop/isaac_rmpflow_stream/status
/factr_teleop/isaac_rmpflow_stream/reason
/factr_teleop/isaac_rmpflow_stream/controller_hz
/factr_teleop/isaac_rmpflow_stream/input_age_ms
/factr_teleop/<side>/isaac_stream_safe_error
```

Bring-up order:

1. Run the offline Lula obstacle-response probe.
2. Run the Isaac stream server and ROS stream bridge against fake/hand-published
   joint states.
3. Run shadow mode on hardware with `publish_safe_targets:=false` and pass
   `collect_stream_shadow_diagnostics.py`.
4. Relaunch the bridge with `publish_safe_targets:=true` and the 0.05 rad
   bring-up limits.
5. Enable FACTR collision-safety consumption at the matched start pose and low
   operator speed.

## Why Not Direct ROS In Isaac First

Direct ROS inside Isaac is possible, but it couples the riskiest parts of the
bring-up at the wrong time:

- Isaac Lab and ROS 2 often want different Python, library, and launch
  environments.
- Importing ROS into Isaac makes startup, graphics/headless mode, and extension
  failures harder to separate from robot safety behavior.
- A crash or stall in Isaac should not take down the ROS process that owns UR
  communication and leader safety checks.
- ZMQ gives a small, testable contract that can be replayed offline before any
  robot is connected.
- The first pass needs transform/schema validation more than tight integration.

Once RMPFlow behavior, frame transforms, and latency are proven, direct ROS in
Isaac can be revisited. The recommended first pass is the two-process ZMQ
boundary because it isolates dependencies and makes stale-output handling
explicit.
