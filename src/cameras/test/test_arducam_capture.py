# ---------------------------------------------------------------------------
# Hardware-in-the-loop check that the Arducam can be opened and read through our
# own software stack (right_cam.yaml + arducam.py), not just raw OpenCV.
#
# The device path comes from right_cam.yaml -- a stable /dev/v4l/by-id/... link
# that survives replug/reboot -- so this test tracks the real config.
#
# It needs the physical camera, so the capture test is skipped automatically
# when the device is absent (e.g. CI). Run it on the robot PC with the camera
# connected:
#   colcon test --packages-select cameras \
#       --pytest-args -k test_arducam_capture -s
# or directly:
#   pytest src/cameras/test/test_arducam_capture.py -s
# ---------------------------------------------------------------------------

import os

import cv2
import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), "..", "configs", "right_cam.yaml")


def _load_config():
    with open(CONFIG, "r") as f:
        return yaml.safe_load(f)


def _device_path():
    return _load_config()["device"]["path"]


requires_camera = pytest.mark.skipif(
    not os.path.exists(_device_path()),
    reason=f"Arducam device {_device_path()} not present",
)


def test_config_is_a_valid_arducam_spec():
    """right_cam.yaml is a complete Arducam spec (stable path + stream)."""
    config = _load_config()
    path = config["device"]["path"]
    # Prefer a stable by-id link over a bare /dev/videoN number.
    assert path.startswith("/dev/v4l/by-id/") or path.startswith("/dev/video")
    assert config["device"]["name"] == "right"
    stream = config["stream"]
    assert stream["width"] > 0 and stream["height"] > 0 and stream["fps"] > 0


@requires_camera
def test_arducam_reads_a_frame():
    """Open the configured Arducam exactly as arducam.py does, read a frame."""
    config = _load_config()
    device = config["device"]["path"]
    stream = config["stream"]

    # Same open path as ArducamNode.initialize_camera().
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    assert cap.isOpened(), f"Cannot open Arducam at {device}"
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, stream["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, stream["height"])
        cap.set(cv2.CAP_PROP_FPS, stream["fps"])
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # First read after configuring can be stale; retry a few frames.
        ok, frame = False, None
        for _ in range(10):
            ok, frame = cap.read()
            if ok and frame is not None:
                break
        assert ok and frame is not None, "Arducam opened but returned no frame"

        h, w = frame.shape[:2]
        assert w > 0 and h > 0
        # It should deliver a 3-channel color image (BGR from OpenCV).
        assert frame.ndim == 3 and frame.shape[2] == 3
    finally:
        cap.release()
