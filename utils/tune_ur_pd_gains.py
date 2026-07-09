#!/usr/bin/env python3
"""Live keyboard tuning of the UR7e follower's direct-torque PD gains.

Runs the real FACTR teleop node (so the follower UR follows the leader arm
exactly as in normal operation) but layers a keyboard tuner on top that mutates
the PD kp/kd gains in place while you teleoperate. Use it to feel out gains, then
copy the printed values back into the config's `controller.torque` block.

    1-6 : select joint (base -> wrist3)
    w/s : selected-joint kp +/-
    e/d : selected-joint kd (damping) +/-
    p   : print current gains
    Ctrl-C : quit (returns the leader arm home and lands the UR, as usual)

Usage (source the ROS overlay first, e.g. `source ./factr_env`):
    python3 utils/tune_ur_pd_gains.py [config_file]
        config_file defaults to ur7e_leader_right.yaml.

Keys are read from /dev/tty directly so this works whether launched from a plain
shell or under `ros2 launch` (where the node's stdin is not the terminal).
"""

import select
import sys
import termios
import threading
import tty

import numpy as np
import rclpy

from factr_teleop.factr_teleop_ur7e import FACTRTeleopUR7e

KP_STEP = 5.0
KD_STEP = 1.0


class FACTRTeleopUR7eTuner(FACTRTeleopUR7e):
    """FACTR UR7e node + a keyboard PD-gain tuner running on a side thread."""

    def _post_match_start(self):
        # Start the normal follower/servo/observation threads, then the tuner.
        super()._post_match_start()
        self._start_gain_tuner()

    def shut_down(self):
        self._stop_gain_tuner()  # restore the terminal before the usual teardown
        super().shut_down()

    # ---------------------------------------------------------------- tuner
    def _start_gain_tuner(self):
        self._tuner_sel = 0
        self._tuner_running = False
        self._tuner_fd = None
        self._tuner_old_term = None
        try:
            self._tuner_fd = open("/dev/tty", "rb", buffering=0)
        except OSError as exc:
            self.get_logger().warn(f"gain tuner: cannot open /dev/tty ({exc}); disabled.")
            return
        self._tuner_old_term = termios.tcgetattr(self._tuner_fd.fileno())
        tty.setcbreak(self._tuner_fd.fileno())  # cbreak keeps ISIG on, so Ctrl-C still works
        self._tuner_running = True
        self._tuner_thread = threading.Thread(target=self._gain_tuner_loop, daemon=True)
        self._tuner_thread.start()
        self.get_logger().info(
            f"gain tuner active: 1-6 select joint, w/s kp +/-{KP_STEP}, "
            f"e/d kd +/-{KD_STEP}, p print."
        )
        self._print_gains()

    def _print_gains(self):
        self.get_logger().info(
            f"[joint {self._tuner_sel + 1}] "
            f"kp={np.round(self.pd_kp, 2).tolist()} "
            f"kd={np.round(self.pd_kd, 2).tolist()}"
        )

    def _gain_tuner_loop(self):
        fd = self._tuner_fd.fileno()
        while self._tuner_running:
            if not select.select([fd], [], [], 0.1)[0]:
                continue
            key = self._tuner_fd.read(1).decode("utf-8", "ignore")
            i = self._tuner_sel
            if key in "123456":
                self._tuner_sel = int(key) - 1
            elif key == "w":
                self.pd_kp[i] += KP_STEP
            elif key == "s":
                self.pd_kp[i] = max(0.0, self.pd_kp[i] - KP_STEP)
            elif key == "e":
                self.pd_kd[i] += KD_STEP
            elif key == "d":
                self.pd_kd[i] = max(0.0, self.pd_kd[i] - KD_STEP)
            elif key != "p":
                continue
            self._print_gains()

    def _stop_gain_tuner(self):
        self._tuner_running = False
        fd_obj = getattr(self, "_tuner_fd", None)
        old_term = getattr(self, "_tuner_old_term", None)
        if fd_obj is not None and old_term is not None:
            try:
                termios.tcsetattr(fd_obj.fileno(), termios.TCSADRAIN, old_term)
            except Exception:
                pass
        if fd_obj is not None:
            try:
                fd_obj.close()
            except Exception:
                pass


def main():
    config_file = sys.argv[1] if len(sys.argv) > 1 else "ur7e_leader_right.yaml"
    # Inject the config as a ROS param override so the base node picks it up.
    rclpy.init(args=["--ros-args", "-p", f"config_file:={config_file}"])
    node = FACTRTeleopUR7eTuner()
    if getattr(node, "leader_match_only", False):
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        while rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down...")
    finally:
        node.shut_down()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
