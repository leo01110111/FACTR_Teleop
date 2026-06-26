# FACTR to Isaac RMPFlow Bridge Plan

Status: draft planning artifact. This document describes the intended bridge
between the existing FACTR UR7e ROS topics and an Isaac-side RMPFlow safety /
motion filter. It does not imply that the bridge is implemented yet.

## Goals

- Keep the UR real-time control loop local to the existing FACTR UR7e node.
- Use the ROS topics FACTR already publishes as the robot-facing contract.
- Put Isaac RMPFlow behind a small bridge boundary so Isaac-specific frame,
  articulation, and transform details do not leak into the teleop loop.
- Return only validated joint-space targets to FACTR before any real robot
  execution consumes them.

## Existing ROS Contract

The UR7e FACTR path already exposes one stream for measured follower state, one
for the operator / leader requested target, and one optional stream for a
safety-filtered target:

| Topic | Direction | Type | Meaning |
|---|---|---|---|
| `/ur/<side>/obs_ur_state` | FACTR UR7e node publishes | `sensor_msgs/JointState` | Current measured UR joint position. `position[:6]` is the real UR arm state in radians. |
| `/factr_teleop/<side>/desired_ur_pos` | FACTR UR7e node publishes | `sensor_msgs/JointState` | Desired UR joint position produced from the matched leader. `position[:6]` is the requested real UR arm target in radians. |
| `/factr_teleop/<side>/safe_ur_pos` | Safety bridge publishes | `sensor_msgs/JointState` | Safety-filtered real UR joint target in radians. The FACTR UR7e node consumes this only when collision safety is enabled. |

`<side>` is `left` or `right`. The joint order is the UR arm order used by
RTDE / `servoJ`:

```text
[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
```

The bridge should treat the current FACTR node as the only component allowed to
talk to RTDE / `servoJ`. Isaac RMPFlow should never command the robot directly.

## Proposed ROS / ZMQ Boundary

The proposed split is:

```text
FACTR ROS nodes
  /ur/<side>/obs_ur_state
  /factr_teleop/<side>/desired_ur_pos
        |
        v
ROS Isaac bridge node
  - subscribes to current and desired joint topics
  - validates freshness and shape
  - applies real-UR to Isaac joint/frame transforms
  - sends compact state/request messages over ZMQ
        |
        v
Isaac RMPFlow process
  - owns Isaac stage, articulation model, base transforms, obstacles
  - computes safe joint targets or pass-through decisions
        |
        v
ROS Isaac bridge node
  - validates the Isaac response
  - converts Isaac joint output back to real UR coordinates
  - publishes /factr_teleop/<side>/safe_ur_pos
```

ROS remains the local hardware integration layer because the existing FACTR UR7e
node already publishes the needed streams and optionally consumes
`safe_ur_pos`. ZMQ is the proposed process / environment boundary for Isaac
because Isaac may run in a separate Python environment or on a separate machine.
The ZMQ boundary should carry policy-rate or safety-rate arrays, not RTDE
control-rate traffic.

Recommended sockets:

- Bridge to Isaac request: `REQ/REP` for the first blocking prototype, then
  migrate to a latest-only `PUB/SUB` or `PUSH/PULL` pair if blocking round trips
  become a problem.
- Isaac diagnostics to bridge or operator tools: optional `PUB/SUB`.
- Message encoding: msgpack or JSON for early bring-up; msgpack once the schema
  stabilizes.

## Data Schema

The bridge sends one bimanual request containing whichever arms are active.
All numeric arrays are float64 radians unless otherwise stated.

```jsonc
{
  "schema": "factr.isaac_rmpflow.request.v1",
  "stamp": 1718900000.123,
  "sequence": 42,
  "active_sides": ["left", "right"],
  "arms": {
    "left": {
      "q_current": [0.0, -1.57, 1.57, -1.57, -1.57, -1.57],
      "q_desired": [0.0, -1.56, 1.56, -1.58, -1.56, -1.57],
      "state_age_s": 0.006,
      "desired_age_s": 0.004
    },
    "right": {
      "q_current": [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0],
      "q_desired": [-1.56, -1.57, -1.58, -1.56, 1.57, 0.02],
      "state_age_s": 0.005,
      "desired_age_s": 0.004
    }
  }
}
```

Response:

```jsonc
{
  "schema": "factr.isaac_rmpflow.response.v1",
  "stamp": 1718900000.130,
  "sequence": 42,
  "ok": true,
  "mode": "filtered",       // pass_through | filtered | hold | stop
  "arms": {
    "left": {
      "q_safe": [0.0, -1.56, 1.56, -1.58, -1.56, -1.57],
      "limit_margin_min": 0.42,
      "distance_margin_min_m": 0.09
    },
    "right": {
      "q_safe": [-1.56, -1.57, -1.58, -1.56, 1.57, 0.02],
      "limit_margin_min": 0.38,
      "distance_margin_min_m": 0.11
    }
  },
  "reason": "rmpflow_clear"
}
```

Schema rules:

- `q_current` comes from `/ur/<side>/obs_ur_state`.
- `q_desired` comes from `/factr_teleop/<side>/desired_ur_pos`.
- `q_safe` is published to `/factr_teleop/<side>/safe_ur_pos`.
- Each arm vector must have exactly 6 joints.
- If either current or desired input is stale, the bridge must publish no new
  `safe_ur_pos` and should report `mode: "hold"` or `mode: "stop"` in
  diagnostics.
- If Isaac fails to respond before `isaac_response_timeout_s`, the bridge must
  publish no new target. Start with `isaac_response_timeout_s: 0.05`, which is
  comfortably below FACTR's `safe_target_timeout` default of `0.25s`; FACTR then
  falls back according to its own safe-target freshness logic.

## Wrist Offsets And Base Transforms

There are two transform categories, and they should remain explicit:

1. Joint-space wrist offsets between the real UR readings and the sim /
   planning model.
2. Rigid base transforms that place each UR arm in the Isaac world.

### Wrist offsets

The current collision-monitor path uses per-side `wrist_3` offsets:

```text
left wrist_3 real-to-sim offset:  pi / 2
right wrist_3 real-to-sim offset: pi
```

The Isaac bridge should preserve that adapter pattern:

```text
q_isaac = q_real
q_isaac[5] += wrist_3_offset[side]

q_real = q_isaac
q_real[5] -= wrist_3_offset[side]
```

The bridge should keep the ROS topics in real UR coordinates. The offset is an
internal bridge detail used only when entering or leaving the Isaac model.

### Base transforms

Isaac RMPFlow needs each arm's base pose in a common world frame. Those base
transforms should be configured once and treated as calibration data, not
recomputed from live joint messages.

Planned bridge configuration:

```yaml
isaac_world:
  left:
    base_frame: left_ur_base
    T_world_base:
      translation_m: [x, y, z]
      quaternion_xyzw: [qx, qy, qz, qw]
  right:
    base_frame: right_ur_base
    T_world_base:
      translation_m: [x, y, z]
      quaternion_xyzw: [qx, qy, qz, qw]
wrist_3_offsets:
  left: 1.57079632679
  right: 3.14159265359
```

Bring-up must include a transform validation step:

- Load the Isaac stage and the bridge transform config.
- Set each articulation to a known real UR joint pose.
- Compare Isaac end-effector poses against measured / expected lab poses.
- Confirm left/right bases are not swapped and no axis mirroring is hidden in
  the joint offset layer.

## Staged Safety Gates

The bridge should advance through these gates before any real robot execution
uses Isaac output:

1. Documentation and schema review.
   Confirm topic names, joint order, per-side wrist offsets, base transforms,
   and timeout behavior.

2. Offline log replay.
   Replay recorded `obs_ur_state` and `desired_ur_pos` messages into the bridge
   without any robot connected. Verify schema validation, transform conversion,
   Isaac response timing, and diagnostics.

3. Isaac-only simulation.
   Run RMPFlow against a simulated scene and confirm `q_safe` is continuous,
   joint-limited, and collision-aware for both pass-through and intervention
   cases.

4. ROS shadow mode on hardware.
   Run FACTR and the bridge while publishing Isaac results only to diagnostics,
   not to `/factr_teleop/<side>/safe_ur_pos`. Compare proposed `q_safe` against
   desired commands and watch for stale data, frame mistakes, or discontinuities.

5. Publish-only safe target.
   Publish `/factr_teleop/<side>/safe_ur_pos`, but keep the FACTR UR7e node
   launched with collision-safety consumption disabled. Confirm topic freshness,
   rates, values, and diagnostics with `ros2 topic echo` / logs.

6. Dry-run consume with robot held still.
   Enable collision-safety consumption only with the robot at the matched start
   pose, low operator motion, and an operator ready to stop. Confirm stale Isaac
   output makes FACTR hold current position rather than chase old targets.

7. Low-speed constrained real execution.
   Use reduced servo gain / speed limits, large RMPFlow margins, and a simple
   workspace. Test one side first, then bimanual. Stop immediately on any
   discontinuity, unexpected wrist wrap, incorrect base transform, or mechanical
   click / skip.

8. Normal teleop trial.
   Only after the above gates pass, tune margins and rates toward normal use.
   Keep hard stop access, FACTR leader matching, follower start-pose checks, and
   UR watchdog behavior enabled.

## Open Decisions

- Whether the first Isaac process should be in-process with ROS Python or
  separate over ZMQ from day one.
- Exact Isaac articulation names and joint order mapping for left and right UR7e
  assets.
- Source of calibrated `T_world_base` values: Isaac stage file, YAML config, or
  a generated calibration artifact.
- Whether `q_safe` should always be a position target or whether later versions
  should allow velocity-limited deltas.
- Diagnostic topic names for Isaac timing, margins, and intervention reasons.

## Proposed Diagnostics

Use these bridge diagnostics for the first ROS prototype:

```text
/factr_teleop/isaac_rmpflow/status
/factr_teleop/isaac_rmpflow/reason
/factr_teleop/isaac_rmpflow/roundtrip_ms
/factr_teleop/isaac_rmpflow/min_distance
/factr_teleop/<side>/isaac_safe_error
```

Keep `/factr_teleop/<side>/safe_ur_pos` as the only command-bearing output.
