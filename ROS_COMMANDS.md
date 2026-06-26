# ROS Commands

## Setup Shell

```bash
cd /home/srianumakonda/FACTR_Teleop
source factr_conda_env
colcon build --packages-select factr_teleop
source install/setup.bash
```

## Right UR7e Leader Match Only

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right_match_only \
  leader_match_only:=true
```

## Left UR7e Leader Match Only

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_left.yaml \
  node_name:=factr_teleop_ur7e_left_match_only \
  leader_match_only:=true
```

## Right UR7e Full FACTR Teleop

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right
```

## Left UR7e Full FACTR Teleop

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_left.yaml \
  node_name:=factr_teleop_ur7e_left
```

## Bimanual UR7e Teleop With Openpi-YAM QP Collision Monitor

Run these in three separate terminals.

Terminal 1, collision monitor:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/ur7e_collision_monitor.py \
  active_sides:=left,right \
  command_mode:=posture \
  rate_hz:=150.0 \
  interarm_activation_distance:=0.15 \
  interarm_release_distance:=0.18
```

Terminal 2, left teleop:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_left.yaml \
  node_name:=factr_teleop_ur7e_left \
  collision_safety:=true
```

Terminal 3, right teleop:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right \
  collision_safety:=true
```

## Right UR7e Teleop With Isaac/Lula RMPFlow Over ZMQ

Before robot bring-up, generate and inspect the Isaac scene:

```bash
cd /home/srianumakonda/FACTR_Teleop
bash scripts/isaac_rmpflow/convert_maxlab_urdf_to_usd.sh

scripts/isaac_rmpflow/run_isaaclab.sh \
  /home/srianumakonda/FACTR_Teleop/scripts/isaac_rmpflow/view_maxlab_primitive_scene.py \
  --headless
```

That writes `generated/isaac_rmpflow/maxlab_dual_ur7e_primitive_scene.usd`
with the MaxLab table/plates/board, dual UR7e base poses, primitive Robotiq
grippers, FACTR initial joint-vector metadata, and Lula collision-sphere
overlays.

## Right UR7e Teleop With High-Rate Isaac/Lula RMPFlow Stream

This is the FACTR-style architecture: the Isaac/Lula process owns a fixed-rate
RMPFlow loop and continuously streams the latest safe target. ROS only feeds
latest observed/desired joint states in and forwards fresh safe targets out.

Run only one active safe-target producer at a time. Do not run the old
OpenPI-YAM collision monitor and the Isaac/Lula stream bridge together while
teleop is consuming `/factr_teleop/<side>/safe_ur_pos`.

Terminal 1, Isaac/Lula streaming controller:

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

Terminal 2, ROS streaming bridge:

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

Keep `publish_safe_targets:=false` only for the shadow run. This observes the
stream and must not publish `/factr_teleop/<side>/safe_ur_pos`. Watch:

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

Go only when the JSON summary has `shadow_healthy: true`,
`safe_ur_pos_counts.right: 0`, fresh right observed/desired topics, fresh left
observed topic for the other-arm obstacle, `controller_hz` present and near the
expected loop rate, low `input_age_ms`, and empty `missing_required_topics` and
`stale_required_topics`. Stop if any active side has nonzero `safe_ur_pos_counts`
while `publish_safe_targets:=false`.

For the first active command path, relaunch Terminal 2 with safe-target
publication enabled and keep the conservative hardware bring-up limits:

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

Then launch teleop in Terminal 3.

For real moving inter-arm validation, keep `--require-other-arm-state` in the
Terminal 1 stream-server command and make sure the left arm is publishing
`/ur/left/obs_ur_state`. Omitting `--require-other-arm-state` is only for
static-left/static-other-arm tests: the server starts with the configured left
initial pose as static obstacles and updates them only if a fresh left observed
state is available.

Terminal 3, right teleop:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right \
  collision_safety:=true \
  safe_target_timeout:=0.10
```

For shadow mode, run the same teleop command without `collision_safety:=true`
so normal FACTR motion continues while the RMPFlow stream is observed.

## Bimanual UR7e With Isaac Sim 6 / cuMotion RMPFlow Stream

This is the parallel Isaac Sim 6 backend. It reuses the same ROS bridge and
FACTR teleop nodes, but swaps Terminal 1 from the Isaac 5.1 Lula server to the
Isaac 6 cuMotion server. Keep the endpoint pair separate from the 5.1 path.

Terminal 1, Isaac 6 / cuMotion:

```bash
cd /home/srianumakonda/FACTR_Teleop

bash scripts/isaac_cumotion/run_cumotion_stream_server.sh \
  --mode rmp \
  --input-endpoint tcp://127.0.0.1:5568 \
  --output-endpoint tcp://127.0.0.1:5569 \
  --loop-hz 500.0 \
  --stale-input-timeout-s 5.0 \
  --policy-sides left,right \
  --dynamic-other-arm-obstacles \
  --require-other-arm-state
```

Terminal 2, ROS stream bridge:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/isaac_rmpflow_stream_bridge.py \
  active_sides:=left,right \
  input_endpoint:=tcp://127.0.0.1:5568 \
  output_endpoint:=tcp://127.0.0.1:5569 \
  publish_hz:=150.0 \
  publish_safe_targets:=true \
  safe_response_timeout_s:=5.0 \
  hold_stale_state:=true \
  hold_stale_desired:=true
```

Terminals 3 and 4 are unchanged from the bimanual FACTR teleop workflow:

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_left.yaml \
  node_name:=factr_teleop_ur7e_left \
  collision_safety:=true \
  safe_target_timeout:=5.0
```

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right \
  collision_safety:=true \
  safe_target_timeout:=5.0
```

## Return Right UR To Initial Match Pose

```bash
ros2 run factr_teleop return_ur_to_initial_match \
  --config-file ur7e_leader_right.yaml
```

## Return Left UR To Initial Match Pose

```bash
ros2 run factr_teleop return_ur_to_initial_match \
  --config-file ur7e_leader_left.yaml
```

## Tune Right Leader Gravity Compensation

```bash
python leader_grav_comp_test.py ur7e_leader_right.yaml
```

## Tune Left Leader Gravity Compensation

```bash
python leader_grav_comp_test.py ur7e_leader_left.yaml
```
