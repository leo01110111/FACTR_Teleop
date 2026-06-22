## Changes

## Set up

Set the inital stance of the UR arm in "arm_teleop/initial_match_joint_pos". This will become the stance you jog the UR arm to every time you launch FACTR.

## Calibration
The leader arm must be in <pose> with the trigger fully closed to calibrate.
<picture>

Note that the factr_teleop records the inital offset calibration and reuses them by reading from the json file, "offsets_{self.config['dynamixel']['leader_name']}.json". If you switch to a new leader arm but use the same configuration, remember to delete that json file. Also each leader-follower set up should have a unique ['dynamixel']['leader_name'] field in their configuration.

Use leader_readout.py to align joint_signs in the yaml file with the UR's joint signs found by manually jogging the UR and seeing what the positive direction is.
The gripper readout (the last element) should decrease as you open the trigger. If not, make the last element of joint_signs -1.
- 1 if joint signs align between the leader and follower.
- -1 if joint signs misalign between the leader and follower.

Use leader_readout.py to record max and min position of the gripper and then calculate actuation range = |max - min|. Then put that value in the config under gripper_teleop/actuation_range. 


## Start up

1. Jog the UR close to the inital pose.
2. Launch FACTR: ros2 launch launch/factr_teleop.py
3. Move the leader arm to the inital stance.


