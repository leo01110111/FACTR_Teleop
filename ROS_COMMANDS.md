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

Run these in three separate terminals. Start in `--mode pass_through` to verify
transport. Use `--mode rmp` only after the Isaac/Lula model has been visually
checked.

Terminal 1, Isaac/Lula server:

```bash
cd /home/srianumakonda/FACTR_Teleop

bash scripts/isaac_rmpflow/run_lula_zmq_server.sh \
  --mode rmp \
  --endpoint tcp://127.0.0.1:5557
```

Terminal 2, ROS/ZMQ bridge:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/isaac_rmpflow_zmq_bridge.py \
  active_sides:=right \
  isaac_endpoint:=tcp://127.0.0.1:5557 \
  request_hz:=100.0 \
  publish_safe_targets:=true
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
