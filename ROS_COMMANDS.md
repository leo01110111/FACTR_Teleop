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
