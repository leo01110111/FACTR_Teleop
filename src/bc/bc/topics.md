# Observation topics recorded by data_record (see record_data.sh).

## State topics (sensor_msgs/JointState)
/ur/left/obs_ur_state
/ur/left/obs_gripper
/ur/left/obs_ur_wrench

## Image topics (sensor_msgs/Image)
/realsense/left/im    # D455 color stream (cameras/realsense node, left_cam.yaml)
/realsense/top/im     # D435 color stream (cameras/realsense node, top_cam.yaml)

## Depth topics (sensor_msgs/Image, 32FC1) -- published but not recorded by default
/realsense/left/depth
/realsense/top/depth
