# Isaac Sim 6 / cuMotion RMPFlow Setup For MaxLab UR7e

This directory is the active Isaac Sim 6 cuMotion bring-up path for FACTR UR7e
collision-aware teleop. The legacy backend has been removed from this checkout.

The current backend uses:

- `env_isaaclab6`
- Isaac Sim 6 `isaacsim.robot_motion.cumotion`
- `configs/isaac_cumotion/maxlab_ur7e_right/robot.urdf`
- `configs/isaac_cumotion/maxlab_ur7e_right/robot.xrdf`
- `configs/isaac_cumotion/maxlab_ur7e_right/rmp_flow.yaml`
- `configs/isaac_cumotion/maxlab_ur7e_scene.yaml`

It has passed headless synthetic request/response smoke testing, but the XRDF
collision spheres, base transforms, wrist offsets, and gains still need Isaac
Sim and hardware validation before treating bimanual collision avoidance as
trusted.

## Stream Server

Terminal 1, Isaac Sim 6 / cuMotion RMPFlow:

```bash
cd /home/srianumakonda/FACTR_Teleop

bash scripts/isaac_cumotion/run_cumotion_stream_server.sh \
  --mode rmp \
  --input-endpoint tcp://127.0.0.1:5568 \
  --output-endpoint tcp://127.0.0.1:5569 \
  --loop-hz 500.0 \
  --stale-input-timeout-s 0.5 \
  --policy-sides left,right \
  --dynamic-other-arm-obstacles \
  --require-other-arm-state
```

Terminal 2, ROS stream bridge:

```bash
cd /home/srianumakonda/FACTR_Teleop
source ./factr_conda_env
source install/setup.bash

ros2 launch launch/isaac_cumotion_stream_bridge.py \
  active_sides:=left,right \
  publish_hz:=250.0 \
  publish_safe_targets:=true \
  state_timeout_s:=0.5 \
  desired_timeout_s:=0.5 \
  safe_response_timeout_s:=0.5 \
  hold_stale_state:=true \
  hold_stale_desired:=true
```

Terminals 3 and 4 are the normal FACTR UR7e teleop nodes with
`collision_safety:=true`.

## Notes

- The ROS bridge is backend-specific now: use `launch/isaac_cumotion_stream_bridge.py`.
- `pyzmq` must be installed in `env_isaaclab6`.
- The stream server is headless Python and does not instantiate `SimulationApp`.
