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
ROS bridge node             ZMQ REQ/REP     Isaac RMPFlow server
- rclpy, ROS 2 topics  ----------------->  - Isaac Lab / Isaac Sim
- FACTR topic contract   joint requests     - Lula / RMPFlow world
- freshness watchdogs    <----------------  - collision scene
- safe_ur_pos publisher    joint response   - no ROS dependency
```

The FACTR process remains the only robot-facing process. It owns ROS, RTDE, UR
startup safety checks, leader matching, and the final `servoJ` command path. The
Isaac process owns Isaac Lab, USD/articulation setup, RMPFlow, and collision
world state. ZMQ is only the boundary between those two environments.

## Process 1: ROS Bridge Node

Run this from the FACTR ROS environment:

```bash
cd /home/srianumakonda/FACTR_Teleop
source factr_conda_env
colcon build --packages-select factr_teleop
source install/setup.bash

ros2 launch launch/isaac_rmpflow_zmq_bridge.py \
  active_sides:=right \
  isaac_endpoint:=tcp://127.0.0.1:5557 \
  request_hz:=100.0 \
  state_timeout_s:=0.10 \
  desired_timeout_s:=0.10 \
  isaac_response_timeout_s:=0.05 \
  publish_safe_targets:=true
```

The bridge should subscribe to the existing FACTR streams:

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

## Process 2: Isaac RMPFlow Server

Run this from the Isaac Lab environment:

```bash
cd /home/srianumakonda/FACTR_Teleop

ISAAC_CONDA_ENV=env_isaaclab \
  bash scripts/isaac_rmpflow/run_lula_zmq_server.sh \
  --mode rmp \
  --endpoint tcp://127.0.0.1:5557 \
  --config-dir /home/srianumakonda/FACTR_Teleop/configs/isaac_rmpflow/maxlab_ur7e_right
```

Use `--mode pass_through` first to test the ZMQ/ROS transport without changing
the desired target except for per-cycle step clipping. Use `--mode rmp` to load
bundled Lula and the generated MaxLab primitive URDF/RMPFlow scaffold. The
server does not import `rclpy`, talk to RTDE, or publish ROS topics.

The current RMPFlow runtime is Isaac/Lula only. The previous experimental
non-Isaac RMP mode was removed from `ur7e_collision_monitor.py`; that node now
keeps only the older position/QP-style safety path with `velocity` and
`posture` modes.

## ZMQ Pattern

Use blocking `REQ/REP` for the first bring-up because it is easy to reason
about and makes one request map to one response. Keep the timeout short enough
that a stuck Isaac frame cannot feed stale targets to the robot.

Recommended defaults:

```text
endpoint: tcp://127.0.0.1:5557
request_hz: 50-100
state_timeout_s: 0.10
desired_timeout_s: 0.10
isaac_response_timeout_s: 0.05
max_response_age_s: 0.10
max_joint_step_rad: 0.05
```

Move to latest-only `PUB/SUB` or `DEALER/ROUTER` only after the first
prototype proves the schema, transforms, and safety behavior.

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
/factr_teleop/isaac_rmpflow/status
/factr_teleop/isaac_rmpflow/reason
/factr_teleop/isaac_rmpflow/roundtrip_ms
/factr_teleop/isaac_rmpflow/min_distance
/factr_teleop/<side>/isaac_safe_error
```

Bring-up order:

1. Run the Isaac server headless and answer synthetic requests.
2. Run the ROS bridge against recorded or hand-published joint states.
3. Run shadow mode on hardware without publishing `safe_ur_pos`.
4. Publish `safe_ur_pos` while FACTR collision-safety consumption is disabled.
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
