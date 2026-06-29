# Policy Inference Stack

Status: **draft / proposal**

This document defines the hardware-interface layer that sits between physical
robots/sensors and the policies that drive them. The goal is a single substrate
that all of these plug into, rather than a separate vertical stack per policy
type.

## The three use cases (what this stack must serve)

Everything below exists to support these three deployment modes. They share one
spine and differ only in **where inference runs** and **whether a learning loop
is closed online**.

1. **VLA models (e.g. openpi `pi0.5`).** Inference is **remote**: the stack
   streams observations to an openpi server and listens for actions. The server
   returns **action chunks** (a horizon), executed async so the arm never stalls
   on inference latency.

2. **Online RL (BC finetuned in the real world).** Same observation/action path
   as the VLA case, **plus** a learning loop: collect transitions
   `(s, a, s', r)` where `r` comes from a **VLM reward node**, a **learner**
   process that updates critic + policy weights on GPU, and a **runner/actor**
   that outputs actions from observations. Actor and learner run as separate
   processes with periodic weight sync.

3. **Vanilla BC.** Like the VLA case, but weights are loaded **locally** in our
   own policy node (no remote server). The simplest backend — build first to
   validate the whole spine.

| | Inference location | Learning online? | Extra components |
|---|---|---|---|
| VLA (pi0.5) | remote (openpi server) | no | openpi client, action-chunk executor |
| Online RL | local actor | **yes** | VLM reward node, replay buffer, learner, weight sync |
| Vanilla BC | local | no | local policy node |

## Goals & principles

1. **Embodiment-agnostic.** Every robot (UR7e, Franka, future arms) is exposed
   through the *same* network contract. A policy should not need to know which
   arm is on the other end beyond a capability descriptor.
2. **Observation-agnostic.** Observations are an open, self-describing dict.
   Force/torque is **optional** — some setups have a wrist F/T sensor, some do
   not. Camera count and names vary. The schema must degrade gracefully.
3. **Clean network boundary.** Each embodiment has a **hardware bridge** that
   exposes *action in* and *state out* over the network. Everything above the
   bridge (obs aggregation, policy, reward, training) is hardware-independent.
4. **The real-time loop is local; the network is policy-rate.** See below — this
   is the load-bearing principle of the whole design.

## The load-bearing principle: rate decoupling

The high-rate control loop (e.g. UR `servoJ`/RTDE at ~500 Hz) lives **inside the
bridge**, local to the robot. It never crosses the network.

The network only carries:

- **state/observations** at policy rate (~10–50 Hz), and
- **action targets / action chunks** at policy rate.

The bridge is responsible for interpolating slow policy targets up to the
robot's control rate (action-chunk buffer + the controller's own smoothing,
e.g. UR servoJ `lookahead_time`/`gain`). Consequences:

- No transport is ever asked to do 500 Hz. Latency/jitter requirements on the
  network are modest.
- A policy can be slow (remote VLA inference at a few Hz) without stalling the
  arm — the bridge keeps moving along the current chunk and the executor
  re-queries before it drains.

## Networking stack: ZMQ (proposed)

**Decision: ZMQ for the policy-rate state/action transport.** Rationale:

- Because the real-time loop is local (above), we are not choosing a transport
  for 500 Hz — we are choosing one for policy-rate messages. The raw-latency
  argument is therefore weak for *both* options.
- Precedent: the existing Franka path already uses ZMQ
  (`factr_teleop_franka_zmq`, addresses in `python_utils/global_configs.py`).
- Explicit message contracts fit the embodiment/observation-agnostic goal
  better than implicit ROS topic conventions.
- Fewer moving parts: no DDS discovery/QoS tuning, no ROS-version coupling
  across the inference machines.

**What we give up vs ROS, and the mitigation:**

| ROS gives free | ZMQ mitigation |
|---|---|
| Camera/arm drivers (ZED, RealSense, Robotiq) | Wrap each vendor SDK in a small publisher once. One-time cost. |
| `message_filters` time-sync | Stamp every message; aggregator syncs by timestamp. |
| `ros2 topic echo` / rqt introspection | Small CLI sniffer that subscribes to the same sockets. |

**Revisit if:** we need many heterogeneous off-the-shelf sensors fast, or
multi-lab interop where ROS is the lingua franca.

## Architecture overview

```
   ┌─────────────┐     ┌─────────────┐     ┌──────────────┐
   │  camera(s)  │     │  hardware   │     │  reward node │   (case 2 only)
   │  publisher  │     │   bridge    │     │  (VLM)       │
   └──────┬──────┘     │ (per arm)   │     └──────┬───────┘
          │ images     └──┬───────▲──┘            │ reward
          │ (pol. rate)   │ state │ action        │
          ▼               ▼       │               ▼
   ┌──────────────────────────────────────────────────────┐
   │              observation aggregator                   │
   │   time-syncs cam + state + (force?) -> obs dict       │
   └───────────────────────┬──────────────────────────────┘
                           │ obs (policy rate)
                           ▼
   ┌──────────────────────────────────────────────────────┐
   │     policy backend  (one of, behind a fixed iface)    │
   │   - BC: local weights                                 │
   │   - VLA: openpi client (remote, action chunks)        │
   │   - RL actor: local, + learner/replay/weight-sync     │
   └───────────────────────┬──────────────────────────────┘
                           │ action / action chunk
                           ▼
                    (back to hardware bridge)
```

The **bridge**, **aggregator**, and **action executor** are the shared spine.
The **policy backend** is the only thing that changes between BC / VLA / RL.

## The hardware bridge contract

Every embodiment implements a bridge that:

1. Owns the local real-time control loop (RTDE/servoJ, vendor RT API, etc.).
2. **Publishes state** at policy rate (PUB socket).
3. **Receives actions** at policy rate (PULL or SUB socket) and executes them,
   interpolating to control rate.
4. Advertises a **capability descriptor** so consumers can adapt:

```jsonc
// capability descriptor (published once on connect / on request)
{
  "embodiment": "ur7e",
  "name": "left",
  "dof": 6,
  "action_space": "joint_position",   // joint_position | joint_delta | ee_pose
  "has_gripper": true,
  "has_force": true,                   // false on setups without F/T
  "control_rate_hz": 500,
  "state_rate_hz": 50
}
```

### State message (out)

```jsonc
{
  "t": 1718900000.123,        // monotonic timestamp (seconds)
  "joint_pos": [...],          // length = dof
  "joint_vel": [...],
  "ee_pose": [x,y,z, qx,qy,qz,qw],   // optional
  "gripper": 0.0,              // normalized 0..1, optional
  "wrench": [fx,fy,fz, tx,ty,tz],    // OPTIONAL — omit if has_force=false
  "safety": { "ok": true }     // estop/limits/force-cutoff status
}
```

### Action message (in)

```jsonc
{
  "t": 1718900000.140,
  "space": "joint_position",   // must match the bridge capability
  "target": [...],             // single target OR ...
  "chunk": [[...], [...]],      // ... a horizon of targets (VLA action chunk)
  "gripper": 1.0
}
```

The bridge **enforces** that `space` matches its capability and that targets are
within joint/workspace/force limits before executing.

## Observation schema (aggregator output)

Open, self-describing, force optional:

```jsonc
{
  "t": 1718900000.123,
  "state": { /* bridge state message */ },
  "images": { "front": "<array>", "wrist": "<array>" },  // names vary per setup
  "instruction": "pick up the cup"   // optional, for VLA
}
```

Aggregator responsibility: collect the latest of each source, reject stale
frames, emit a synchronized obs at policy rate. Force simply isn't a key when
the bridge reports `has_force=false`.

## Policy backend interface

```python
class Policy:
    def reset(self): ...                 # episode reset hook (RL/eval)
    def act(self, obs: dict) -> dict:    # returns an action message
        ...
```

- **BC** (case 3): local weights, single forward pass. Build first — smoke-tests
  the whole spine with no network dependency.
- **VLA** (case 1): `act()` is an openpi client; handles **action chunking**
  (consume a horizon, async re-query before drain) and image/instruction
  formatting.
- **RL actor** (case 2): local actor + separate **learner** process (GPU),
  shared **replay buffer**, periodic **weight push** learner→actor. Reward
  arrives async from the VLM node. Never train in the control loop.

## Serialization & transport details (to finalize)

- **Serialization:** msgpack for state/action (compact, language-agnostic).
  Images: JPEG/PNG-encode for cross-host; consider IPC/shared-memory for
  same-host camera→aggregator to avoid copy overhead.
- **Socket patterns:** PUB/SUB for state & images (fan-out, drop-old-ok),
  PUSH/PULL or latest-only SUB for actions. Avoid REQ/REP on the hot path
  (lock-step round-trips add latency).
- **Addressing:** centralized in `global_configs.py` (as today), per
  embodiment/arm: ip + ports for state, action, each camera.
- **Time:** every message carries a timestamp; the aggregator is the single
  point of sync. Decide on a shared clock for cross-host setups.

## Build order

1. Extract UR follower I/O from `factr_teleop_ur7e.py` into a shared `ur_bridge`.
2. Observation aggregator (camera + state + optional force, timestamp-synced).
3. Action executor / chunk buffer (slow policy → smooth control-rate output).
4. **BC backend** end-to-end as the spine smoke test.
5. **VLA backend** (openpi client + async chunking).
6. **Online RL** (reward node, replay buffer, actor/learner split, resets).

## Open decisions

- **Action representation** shared across all three policy backends
  (joint_position vs joint_delta vs ee_pose). Must be consistent end-to-end —
  this is the #1 silent-failure risk.
- **Where the RL learner runs** (same box as actor vs separate machine).
- **Episode reset strategy** for online RL (scripted `moveJ` home vs human).
- **Image transport** same-host (shared memory?) vs cross-host (compressed).
- **Single vs dual-arm** deployment — per-arm bridges vs combined.
- **Safety authority** in autonomous mode (force cutoff via wrist F/T, workspace
  limits, e-stop path) — no human-on-leader fallback once teleop is removed.
