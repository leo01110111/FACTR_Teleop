"""Minimal Direct Joint Torque Control test for the UR7e (right arm).

directTorque() consumes exactly one control step regardless of speed scaling
(like sync()). It MUST be called every robot control cycle (500 Hz -> 2 ms),
otherwise the robot falls back to position control mode.

Gravity is compensated internally; friction compensation is on by default and
can be disabled with friction_comp=False.

Live per-joint tuning:
  1-6 : select the joint to tune
  w/s : raise / lower kp for the selected joint
  e/d : raise / lower kd for the selected joint
  p   : print current gains
  q   : quit
Input is read non-blocking so the control loop never stalls.
"""

import sys
import select
import termios
import tty

import numpy as np
import rtde_control
import rtde_receive

ROBOT_IP = "192.168.2.2"     # right arm (ur_right_ip_address)
DT = 1.0 / 125.0             # FACTR control period 0.05
FREQUENCY = 1.0 / DT
UR_CAP_PORT = 50003          # ExternalControl URCap "Custom port" on the right arm

# Connect through the ExternalControl URCap so the pendant can stay in Local
# (manual) mode instead of Remote Control. The ExternalControl program must be
# running (press PLAY) with Host IP = this PC and Custom port = UR_CAP_PORT.
rtde_c = rtde_control.RTDEControlInterface(
    ROBOT_IP, FREQUENCY,
    rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP, UR_CAP_PORT,
)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
gains = [138, 80]
set_point = np.array([-1.5700, -1.5700, -1.5700, -1.5700, 1.5700, 1.57])

# Per-joint gains (base -> wrist3). Tune each joint independently.
kp = np.array([160.0, 160.0, 160, 130, 100, 80])
kd = np.array([30.0, 30.0, 30.0, 30.0, 1.0, 1.0])

# Per-joint torque clamp [Nm] so a bad gain can't slam the arm / trip a stop.
TAU_MAX = np.array([50.0, 50.0, 30.0, 12.0, 12.0, 12.0])

KP_STEP = 0.1
KD_STEP = 0.1

sel = 0   # index of the joint currently being tuned (0..5)


def read_key():
    """Return a pending keypress without blocking, or None if none is ready."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def print_gains():
    print(f"[joint {sel + 1}]  "
          f"kp={np.round(kp, 3).tolist()}  "
          f"kd={np.round(kd, 3).tolist()}", end="\r\n")


# Put the terminal in cbreak mode so single keys arrive without Enter.
old_term = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())
try:
    print_gains()
    while True:
        # initPeriod/waitPeriod keep the loop locked to the robot control step.
        t_start = rtde_c.initPeriod()

        key = read_key()
        if key is not None:
            if key in "123456":
                sel = int(key) - 1
                print_gains()
            elif key == "w":
                kp[sel] += KP_STEP
                print_gains()
            elif key == "s":
                kp[sel] = max(0.0, kp[sel] - KP_STEP)
                print_gains()
            elif key == "e":
                kd[sel] += KD_STEP
                print_gains()
            elif key == "d":
                kd[sel] = max(0.0, kd[sel] - KD_STEP)
                print_gains()
            elif key == "p":
                print_gains()
            elif key == "q":
                break

        # Joint-space PD control law. Gravity is already compensated internally.
        q = np.array(rtde_r.getActualQ())
        dq = np.array(rtde_r.getActualQd())
        ctrl = kp * (set_point - q) - kd * dq
        ctrl = np.clip(ctrl, -TAU_MAX, TAU_MAX)

        rtde_c.directTorque(ctrl.tolist(), friction_comp=True)

        rtde_c.waitPeriod(t_start)   # sleeps the remainder of DT
except KeyboardInterrupt:
    pass
finally:
    # Restore the terminal, zero the torque, hand control back cleanly.
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
    rtde_c.directTorque([0.0] * 6, friction_comp=True)
    rtde_c.stopScript()
    rtde_c.disconnect()
    rtde_r.disconnect()
