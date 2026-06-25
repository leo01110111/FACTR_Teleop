#!/usr/bin/env python3
"""
Robotiq 2F-85 current / state probe.

Talks to the Robotiq URCap socket (port 63352, opened by the patched
factrfw URCap) and characterizes the gripper in three states so we can pick a
sensible grasp threshold / dead-band for the haptic feedback:

  1. OPEN, nothing in hand
  2. CLOSED, nothing in hand   (fingers stall on each other)
  3. CLOSED, holding an object (fingers stall on the object)

For each state it reports motor current (COU, 0..255), position (POS), and the
object-detection status (OBJ). OBJ is the important one:
    OBJ = 3  -> reached the requested position (nothing detected)
    OBJ = 2  -> stopped early while closing (object detected) == HOLDING
    OBJ = 1  -> stopped early while opening
    OBJ = 0  -> still moving

Usage:
    python3 gripper_current_probe.py                 # guided 3-state characterization
    python3 gripper_current_probe.py --stream        # continuous live readout
    python3 gripper_current_probe.py --ip 192.168.2.2 --port 63352

Run with FACTR NOT running (only one client should drive the gripper socket).
"""
import argparse
import socket
import statistics
import time


class Gripper:
    def __init__(self, ip, port=63352, timeout=2.0):
        self.sock = socket.create_connection((ip, port), timeout=timeout)

    def cmd(self, c):
        self.sock.sendall((c + "\n").encode())
        return self.sock.recv(1024).decode().strip()

    def get(self, name):
        # response looks like "COU 12"
        return int(self.cmd(f"GET {name}").split()[1])

    def set_pos(self, pos):
        self.cmd(f"SET POS {max(0, min(255, int(pos)))}")

    def activate(self, speed, force, wait_timeout=5.0):
        self.cmd("SET ACT 1")
        self.cmd(f"SET SPE {max(0, min(255, int(speed)))}")
        self.cmd(f"SET FOR {max(0, min(255, int(force)))}")
        self.cmd("SET GTO 1")
        t0 = time.time()
        while self.get("STA") != 3 and time.time() - t0 < wait_timeout:
            time.sleep(0.1)


def sample(g, seconds=1.5, hz=20.0):
    """Sample COU/POS/OBJ for `seconds` after the gripper has settled."""
    period = 1.0 / hz
    cou, pos, obj = [], [], []
    t0 = time.time()
    while time.time() - t0 < seconds:
        cou.append(g.get("COU"))
        pos.append(g.get("POS"))
        obj.append(g.get("OBJ"))
        time.sleep(period)
    return cou, pos, obj


def summarize(label, cou, pos, obj):
    cmin, cmax = min(cou), max(cou)
    cmean = statistics.mean(cou)
    # OBJ is categorical; report the most common value.
    obj_mode = statistics.mode(obj)
    print(f"\n=== {label} ===")
    print(f"  current COU : min {cmin:3d}  mean {cmean:6.1f}  max {cmax:3d}   "
          f"(normalized mean {cmean/255.0:.3f})")
    print(f"  position POS: min {min(pos):3d}  max {max(pos):3d}")
    print(f"  object  OBJ : {obj_mode}  "
          f"({'HOLDING (stopped on object)' if obj_mode == 2 else 'reached position / no object' if obj_mode == 3 else 'moving/other'})")
    return {"label": label, "cou_mean": cmean, "cou_max": cmax, "obj": obj_mode}


def guided(g, settle=1.5):
    results = []

    input("\n[1/3] OPEN, nothing in hand. Remove any object, then press Enter...")
    g.set_pos(0)            # 0 = open
    time.sleep(settle)
    results.append(summarize("OPEN, nothing", *sample(g)))

    # Holding must come before closed-empty: the gripper has to be OPEN (from the
    # step above) so an object can be placed between the fingers before closing.
    input("\n[2/3] HOLDING. Place an object between the open fingers, then press Enter...")
    g.set_pos(255)          # close onto the object
    time.sleep(settle)
    results.append(summarize("CLOSED, holding object", *sample(g)))

    print("\nOpening so you can remove the object...")
    g.set_pos(0)            # open back up to free the object
    time.sleep(settle)
    input("[3/3] CLOSED, nothing. Remove the object, then press Enter...")
    g.set_pos(255)          # close onto nothing
    time.sleep(settle)
    results.append(summarize("CLOSED, nothing", *sample(g)))

    # ---- recommendation -------------------------------------------------
    by = {r["label"]: r for r in results}
    print("\n================ summary ================")
    for r in results:
        print(f"  {r['label']:24s} COU mean {r['cou_mean']:6.1f}  max {r['cou_max']:3d}  OBJ {r['obj']}")

    closed_empty = by["CLOSED, nothing"]["cou_mean"]
    holding = by["CLOSED, holding object"]["cou_mean"]
    open_empty = by["OPEN, nothing"]["cou_mean"]
    print("\n---------------- analysis ----------------")
    print(f"  open-empty current : {open_empty:.1f}")
    print(f"  closed-empty current: {closed_empty:.1f}")
    print(f"  holding current     : {holding:.1f}")
    if abs(holding - closed_empty) < 0.15 * 255:
        print("  >> closed-empty and holding currents are CLOSE: current alone")
        print("     cannot reliably tell 'holding' from 'closed on nothing'.")
        print("     Use OBJ status instead (OBJ==2 => holding) to gate feedback.")
    else:
        mid = (holding + closed_empty) / 2.0
        print(f"  >> currents separable. A current threshold ~{mid:.0f} (norm {mid/255.0:.3f})")
        print(f"     distinguishes holding from idle; use it as the feedback dead-band.")
    print("  Note: OBJ is usually the cleaner grasp signal (2=holding, 3=no object).")


def stream(g, hz=10.0):
    print("Live readout (Ctrl-C to stop). Drive the gripper by hand/pendant/FACTR.")
    print(f"{'POS':>5} {'COU':>5} {'COU/255':>8} {'OBJ':>4}")
    period = 1.0 / hz
    try:
        while True:
            print(f"{g.get('POS'):5d} {g.get('COU'):5d} {g.get('COU')/255.0:8.3f} {g.get('OBJ'):4d}")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopped.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.1.2")
    ap.add_argument("--port", type=int, default=63352)
    ap.add_argument("--speed", type=int, default=255)
    ap.add_argument("--force", type=int, default=128)
    ap.add_argument("--stream", action="store_true", help="continuous readout instead of guided")
    ap.add_argument("--no-activate", action="store_true", help="skip activation (already active)")
    args = ap.parse_args()

    print(f"Connecting to gripper at {args.ip}:{args.port} ...")
    g = Gripper(args.ip, args.port)
    if not args.no_activate:
        print("Activating (this moves the gripper)...")
        g.activate(args.speed, args.force)
    print("Connected.")

    if args.stream:
        stream(g)
    else:
        guided(g)


if __name__ == "__main__":
    main()
