#!/usr/bin/env python3
"""Open the generated MaxLab dual-UR7e USD scene in Isaac Sim.

Run through Isaac Lab's launcher, for example:
  cd /home/srianumakonda/IsaacLab
  TERM=xterm ./isaaclab.sh -p /home/srianumakonda/FACTR_Teleop/scripts/isaac_rmpflow/view_maxlab_usd.py
"""

from __future__ import annotations

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher


DEFAULT_USD = Path("/home/srianumakonda/FACTR_Teleop/generated/isaac_rmpflow/maxlab_dual_ur7e_table.usd")

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--usd", type=Path, default=DEFAULT_USD)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import carb
import omni.kit.app

import isaaclab.sim as sim_utils


def main() -> None:
    usd_path = args_cli.usd.resolve()
    if not usd_path.exists():
        raise FileNotFoundError(f"USD does not exist yet: {usd_path}")

    sim_utils.open_stage(str(usd_path))
    print(f"[INFO] Opened MaxLab UR7e scene: {usd_path}")
    print("[INFO] Expected base poses from MaxLab scene:")
    print("  left  pos=[-0.7425, -0.005, 0.766], yaw=+90deg")
    print("  right pos=[ 0.7425, -0.005, 0.766], yaw=-90deg")
    print("[INFO] FACTR real-to-sim wrist_3 offsets: left=pi/2, right=pi")

    app = omni.kit.app.get_app_interface()
    settings = carb.settings.get_settings()
    local_gui = bool(settings.get("/app/window/enabled"))
    livestream_gui = bool(settings.get("/app/livestream/enabled"))
    if not local_gui and not livestream_gui:
        for _ in range(10):
            app.update()
        return

    while app.is_running():
        app.update()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
