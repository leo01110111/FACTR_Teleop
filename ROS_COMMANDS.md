# ROS Commands

## Setup Shell

```bash
cd /home/srianumakonda/FACTR_Teleop
source factr_conda_env
colcon build --packages-select factr_teleop
source install/setup.bash
```

## Leader Match Only

Right:

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right_match_only \
  leader_match_only:=true
```

Left:

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_left.yaml \
  node_name:=factr_teleop_ur7e_left_match_only \
  leader_match_only:=true
```

## Plain FACTR Teleop

Right:

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right
```

Left:

```bash
ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_left.yaml \
  node_name:=factr_teleop_ur7e_left
```

## Bimanual UR7e With OpenPI-YAM QP Collision Monitor

Run these in three separate terminals. Do not run this collision monitor at the
same time as an Isaac cuMotion bridge that publishes `/factr_teleop/<side>/safe_ur_pos`.

Terminal 1:

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

Terminal 2:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_left.yaml \
  node_name:=factr_teleop_ur7e_left \
  collision_safety:=true
```

Terminal 3:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right \
  collision_safety:=true
```

## Bimanual UR7e With Isaac Sim 6 / cuMotion RMPFlow

This is the active Isaac backend. The cuMotion server runs in `env_isaaclab6`
and communicates with ROS through ZMQ. The ROS bridge publishes the same
`/factr_teleop/<side>/safe_ur_pos` topics consumed by `collision_safety:=true`.

Use `configs/isaac_cumotion/README.md` for the full cuMotion server, ROS bridge,
teleop, and diagnostic commands.

## Return UR To Initial Match Pose

Right:

```bash
ros2 run factr_teleop return_ur_to_initial_match \
  --config-file ur7e_leader_right.yaml
```

Left:

```bash
ros2 run factr_teleop return_ur_to_initial_match \
  --config-file ur7e_leader_left.yaml
```

## Tune Leader Gravity Compensation

Right:

```bash
python leader_grav_comp_test.py ur7e_leader_right.yaml
```

Left:

```bash
python leader_grav_comp_test.py ur7e_leader_left.yaml
```
