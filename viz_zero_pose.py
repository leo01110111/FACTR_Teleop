#!/usr/bin/env python3
"""Visualize the UR5-GELLO leader URDF at the all-zeros pose (Meshcat in browser).
Used to find the physical calibration stance for calibration_joint_pos = [0]*6."""
import time
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import os

URDF = "src/factr_teleop/factr_teleop/urdf/factr_teleop_ur5_gello.urdf"
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    filename=URDF, package_dirs=os.path.dirname(URDF)
)
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=False)
viz.loadViewerModel()

q0 = np.zeros(model.nq)          # the calibration "all zeros" pose
viz.display(q0)
viz.displayFrames(True)          # show joint frames to read orientations

print("\n==================================================")
print(" MESHCAT URL ->", viz.viewer.url())
print(" Arm shown at q = [0, 0, 0, 0, 0, 0]")
print(" Open that URL in a browser. Ctrl-C here to stop.")
print("==================================================\n", flush=True)

while True:
    time.sleep(1)
