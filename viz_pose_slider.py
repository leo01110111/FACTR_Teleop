#!/usr/bin/env python3
"""Interactive joint sliders (tkinter) + meshed arm in Meshcat (browser).
Drag joints to the desired calibration stance; the current q is written to
/tmp/cal_pose.txt continuously and printed."""
import os, numpy as np, tkinter as tk
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

URDF = "src/factr_teleop/factr_teleop/urdf/factr_teleop_ur5_gello.urdf"
OUT = "/tmp/cal_pose.txt"
model, cmodel, vmodel = pin.buildModelsFromUrdf(URDF, package_dirs=os.path.dirname(URDF))
joint_names = [n for n in model.names if n != "universe"]

viz = MeshcatVisualizer(model, cmodel, vmodel)
viz.initViewer(open=False)
viz.loadViewerModel()
print("\n  MESHCAT URL ->", viz.viewer.url(), "\n", flush=True)

q = np.zeros(model.nq)
viz.display(q)

root = tk.Tk()
root.title("UR5-GELLO joint sliders")
val_label = tk.Label(root, text="", font=("monospace", 11), justify="left")
val_label.pack(padx=8, pady=6)

def update(_=None):
    for i, s in enumerate(sliders):
        q[i] = s.get()
    viz.display(q)
    txt = "  ".join(f"{v:+.3f}" for v in q)
    val_label.config(text=f"q = [{txt}]")
    with open(OUT, "w") as f:
        f.write(" ".join(f"{v:.6f}" for v in q) + "\n")

sliders = []
for name in joint_names:
    row = tk.Frame(root); row.pack(fill="x", padx=8)
    tk.Label(row, text=name, width=22, anchor="w").pack(side="left")
    s = tk.Scale(row, from_=-3.2, to=3.2, resolution=0.01, orient="horizontal",
                 length=420, command=update)
    s.set(0.0); s.pack(side="left")
    sliders.append(s)

tk.Button(root, text="Reset to 0", command=lambda: ([s.set(0.0) for s in sliders], update())).pack(pady=6)
update()
print("Sliders ready. Drag to the desired pose; q is written to", OUT, flush=True)
root.mainloop()
