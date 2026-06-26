# Isaac/Lula RMPFlow Setup For MaxLab UR7e

This folder tracks the Isaac-side setup for FACTR dual-UR7e collision avoidance.
Use the MaxLab assets as source of truth, not stock Isaac UR assets.

## Source Assets

- `/home/srianumakonda/maxlab/sim/build_urtable.py`
- `/home/srianumakonda/maxlab/sim/universal_robots_ur5e/ur5e.xml`
- `/home/srianumakonda/maxlab/sim/gripper/robotiq-2f85.xml`

The `ur5e` path is legacy naming. MaxLab comments say to treat the arm model as
UR7e in this project.

## Primitive URDF/USD Path

The Isaac MJCF importer currently creates an empty stage from the full MaxLab
MJCF scene in this environment, so the working bring-up path is a generated
primitive URDF derived from the MaxLab UR MJCF collision geometry, plus a
fixed primitive Robotiq 2F-85 envelope derived from the MaxLab gripper layout.

Generate the primitive URDF and convert it to USD:

```bash
cd /home/srianumakonda/FACTR_Teleop
bash scripts/isaac_rmpflow/convert_maxlab_urdf_to_usd.sh
```

Write/open a primitive dual-arm scene at the MaxLab base poses:

```bash
cd /home/srianumakonda/FACTR_Teleop
scripts/isaac_rmpflow/run_isaaclab.sh \
  /home/srianumakonda/FACTR_Teleop/scripts/isaac_rmpflow/view_maxlab_primitive_scene.py --headless
```

The generated files are:

```text
configs/isaac_rmpflow/maxlab_ur7e_right/maxlab_ur7e_right.urdf
generated/isaac_rmpflow/maxlab_ur7e_right_primitive.usd
generated/isaac_rmpflow/maxlab_dual_ur7e_primitive_scene.usd
```

The scene writer includes the MaxLab table, robot base plates, board pieces,
dual UR7e references, fixed Robotiq envelopes, and translucent Lula collision
sphere overlays. It also stores the FACTR real and wrist-offset sim initial
joint vectors as `factr:initial_q_real` and `factr:initial_q_sim` attributes on
`/World/left_ur7e` and `/World/right_ur7e`.

## Isaac/Lula RMPFlow ZMQ Server

The RMPFlow server uses bundled Lula directly and communicates with FACTR ROS
through ZMQ. Use `pass_through` mode first to test transport, then `rmp` mode to
exercise Lula/RMPFlow.

```bash
cd /home/srianumakonda/FACTR_Teleop
bash scripts/isaac_rmpflow/run_lula_zmq_server.sh \
  --mode rmp \
  --endpoint tcp://127.0.0.1:5557
```

## MJCF Importer Debug Path

The original full-scene MJCF export/conversion scripts are still present for
debugging Isaac's MJCF importer, but they are not the current RMPFlow runtime
path:

```bash
cd /home/srianumakonda/FACTR_Teleop
python scripts/isaac_rmpflow/export_maxlab_mjcf.py
bash scripts/isaac_rmpflow/convert_maxlab_mjcf_to_usd.sh
```

## Lula/RMPFlow Files

Lula RMPFlow does not use USD alone. It expects:

- a robot URDF for kinematics,
- a robot descriptor YAML for collision spheres / active joints,
- an RMPFlow config YAML,
- an end-effector frame name.

The generated primitive URDF is the current kinematic input for Lula. The
remaining work is validation: confirm axes, limits, wrist offsets, collision
spheres, base poses, and RMPFlow gains before using RMPFlow on hardware.

## Right UR7e Scaffold

`maxlab_ur7e_right/` contains a starting Lula/RMPFlow scaffold for the right
MaxLab UR7e, following Isaac Sim's installed UR5e example layout:

- `maxlab_ur7e_right.urdf`
- `rmpflow/config.json`
- `rmpflow/maxlab_ur7e_right_robot_description.yaml`
- `rmpflow/maxlab_ur7e_right_rmpflow_config.yaml`
- `maxlab_ur7e_right_metadata.yaml`

This is not hardware-verified. The URDF is generated from the MaxLab MJCF
kinematic/collision model, while the RMPFlow gains, collision spheres, and
initial-pose articulation still need visual/operator validation before hardware
use. The metadata records the right base pose and the known right `wrist_3`
real-to-sim offset of `pi` from the MaxLab scene setup.
