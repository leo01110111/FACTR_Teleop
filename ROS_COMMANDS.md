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

## Right UR7e RMP Free-Space Tracking

Use this when testing RMP tracking without collision avoidance. The active
`configs/isaac_cumotion/maxlab_ur7e_right/rmp_flow.yaml` should have
`collision_rmp/metric_scalar: 0.` for this test.

Terminal 1: start the cuMotion RMP server.

```bash
cd /home/srianumakonda/FACTR_Teleop
./scripts/isaac_cumotion/run_cumotion_stream_server.sh \
  --mode rmp
```

Terminal 2: start the ROS bridge and publish RMP safe targets.

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/isaac_cumotion_stream_bridge.py \
  active_sides:=right \
  publish_safe_targets:=true \
  require_rmp_policy:=true \
  max_joint_step_rad:=0.2 \
  max_safe_target_distance_rad:=1.0
```

Terminal 3: start right teleop using the RMP safe targets.

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/factr_teleop_ur7e.py \
  config_file:=ur7e_leader_right.yaml \
  node_name:=factr_teleop_ur7e_right \
  collision_safety:=true \
  safe_target_timeout:=0.25
```

Diagnostic terminal:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 topic hz /factr_teleop/right/safe_ur_pos
ros2 topic echo /factr_teleop/isaac_cumotion_stream/status
ros2 topic echo /factr_teleop/right/isaac_cumotion_safe_error
```

Quick health check after the leader has calibrated:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
ros2 node list
ros2 topic list | sort
timeout 5 ros2 topic hz /factr_teleop/right/desired_ur_pos
timeout 5 ros2 topic hz /factr_teleop/right/safe_ur_pos
timeout 5 ros2 topic echo /factr_teleop/isaac_cumotion_stream/status
timeout 5 ros2 topic echo /factr_teleop/right/isaac_cumotion_safe_error
```

Expected result:

```text
/factr_teleop_ur7e_right is present
/isaac_cumotion_stream_bridge is present
/factr_teleop/right/desired_ur_pos is publishing after leader match
/factr_teleop/right/safe_ur_pos is publishing near the bridge rate
/factr_teleop/isaac_cumotion_stream/status reports active/ok, not waiting_response
```

If the leader calibrates but the follower does not move, first check
`/factr_teleop/right/safe_ur_pos`. With `collision_safety:=true`, the teleop node
holds the current UR pose whenever the safe target is missing or stale.
Keep the teleop process running while checking this; `Ctrl-\` sends SIGQUIT and
kills the process (`Quit (core dumped)`). Use `Ctrl-C` for normal shutdown.

The teleop terminal should print:

```text
FACTR UR7e right: first safe_ur_pos received.
```

If it instead prints one of these, the arm is intentionally holding:

```text
FACTR UR7e right: holding current pose because no safe_ur_pos has been received.
FACTR UR7e right: holding current pose because safe_ur_pos is stale (...s old).
```

## Left UR7e RMP Test At 500 Hz

Run these in four separate terminals. Use this when testing the left follower
against the right arm as a static dynamic obstacle.

Terminal 1: publish the right UR state as the obstacle at 500 Hz.

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

python - <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rtde_receive import RTDEReceiveInterface

class StaticRightURState(Node):
    def __init__(self):
        super().__init__("right_ur_static_state_500hz")
        rtde_r = RTDEReceiveInterface("192.168.2.2")
        self.q = list(rtde_r.getActualQ())[:6]
        rtde_r.disconnect()
        self.pub = self.create_publisher(JointState, "/ur/right/obs_ur_state", 10)
        self.create_timer(1.0 / 500.0, self.tick)

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = self.q
        self.pub.publish(msg)

rclpy.init()
node = StaticRightURState()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

Terminal 2: run the cuMotion/RMP server.

```bash
cd /home/srianumakonda/FACTR_Teleop

bash scripts/isaac_cumotion/run_cumotion_stream_server.sh \
  --mode rmp \
  --loop-hz 500.0 \
  --stale-input-timeout-s 0.5 \
  --policy-sides left \
  --dynamic-other-arm-obstacles \
  --require-other-arm-state
```

Terminal 3: run the ROS bridge.

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 run factr_teleop isaac_cumotion_stream_bridge --ros-args \
  -p active_sides:=left \
  -p publish_hz:=500.0 \
  -p publish_safe_targets:=true \
  -p safe_response_timeout_s:=0.25 \
  -p state_timeout_s:=0.20 \
  -p desired_timeout_s:=0.20 \
  -p max_joint_step_rad:=0.08 \
  -p max_safe_target_distance_rad:=0.08 \
  -p max_sequence_lag:=5 \
  -p hold_stale_state:=true \
  -p hold_stale_desired:=true
```

Terminal 4: run left teleop with collision safety.

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 run factr_teleop factr_teleop_ur7e --ros-args \
  -r __node:=factr_teleop_ur7e_left \
  -p config_file:=ur7e_leader_left.yaml \
  -p collision_safety:=true \
  -p safe_target_timeout:=0.25
```

Diagnostic terminal: check the actual rates.

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

timeout 10 ros2 topic echo /factr_teleop/left/servo_hz
timeout 10 ros2 topic echo /factr_teleop/left/observation_hz
timeout 10 ros2 topic echo /factr_teleop/left/leader_hz
timeout 10 ros2 topic echo /factr_teleop/isaac_cumotion_stream/controller_hz
timeout 10 ros2 topic hz /factr_teleop/left/safe_ur_pos
```

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
