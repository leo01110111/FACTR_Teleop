# Isaac Sim 6 / cuMotion RMPFlow Setup For MaxLab UR7e

This directory is the Isaac Sim 6 cuMotion bring-up path. It intentionally
coexists with the Isaac 5.1 Lula setup under `configs/isaac_rmpflow/`.

The current scaffold uses:

- `env_isaaclab6`
- Isaac Sim 6 `isaacsim.robot_motion.cumotion`
- the generated MaxLab UR7e primitive URDF from the 5.1 bring-up
- cuMotion's stock UR10 XRDF/RMPFlow config shape as a starting point

It has passed a headless synthetic bimanual request/response smoke test, but it
has not been validated on hardware. Treat it as a validation-stage backend until
the frames, collision spheres, and gains are checked.

## Stream Server

Terminal 1, Isaac 6 / cuMotion RMPFlow:

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

Terminal 2, the same ROS stream bridge, pointed at the cuMotion ports:

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

Terminals 3 and 4 are the normal FACTR UR7e teleop nodes with
`collision_safety:=true`.

## Notes

- The ROS bridge is backend-agnostic. It only needs matching ZMQ endpoints.
- Do not run the 5.1 Lula stream server and this 6.0 cuMotion stream server on
  the same endpoint pair.
- `pyzmq` is installed in `env_isaaclab6` for this ZMQ server.
- The cuMotion server is pure headless Python and does not instantiate
  `SimulationApp`.
