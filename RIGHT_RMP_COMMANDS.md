# Right UR7e RMP Free-Space Commands

## Terminal 1: cuMotion RMP Server

```bash
cd /home/leo/factr_teleop_ur7e
./scripts/isaac_cumotion/run_cumotion_stream_server.sh \
  --mode rmp
```

## Terminal 2: ROS Bridge

```bash
cd /home/leo/factr_teleop_ur7e
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/isaac_cumotion_stream_bridge.py \
  active_sides:=right \
  publish_safe_targets:=true \
  require_rmp_policy:=true \
  max_joint_step_rad:=0.2 \
  max_safe_target_distance_rad:=1.0
```

## Terminal 3: Right Teleop

```bash
cd /home/leo/factr_teleop_ur7e
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right \
  collision_safety:=true \
  safe_target_timeout:=0.25
```

## Quick Checks

```bash
cd /home/leo/factr_teleop_ur7e
source ./factr_conda_env
source install/setup.bash

ros2 node list
ros2 topic hz /factr_teleop/right/desired_ur_pos
ros2 topic hz /factr_teleop/right/safe_ur_pos
ros2 topic hz /ur/right/obs_ur_state
ros2 topic echo /factr_teleop/isaac_cumotion_stream/status
ros2 topic echo /factr_teleop/isaac_cumotion_stream/reason
ros2 topic echo /factr_teleop/right/isaac_cumotion_safe_error
```
