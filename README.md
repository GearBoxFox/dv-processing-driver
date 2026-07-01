# dv-processing-driver

Updated ROS 2 driver for the iniVtion DVXplorer Micro using the `dv-processing` library.

## Overview

This package provides a ROS 2 node that publishes events, IMU data and accumulated frames from a DVXplorer camera using the `dv-processing` C++ library.

## Requirements

- ROS 2 (install and source your distro, e.g. `source /opt/ros/<ros2-distro>/setup.bash`)
- `dv-processing` library (the native camera driver used by this package)
- OpenCV, Eigen3, pugixml and libusb development packages
- colcon build tool for building the workspace

On Debian/Ubuntu you can usually install common dependencies with apt (replace `<ros2-distro>`):

```bash
sudo apt update
sudo apt install -y build-essential cmake git \
	libopencv-dev libeigen3-dev libpugixml-dev libusb-1.0-0-dev \
	python3-colcon-common-extensions \
	ros-<ros2-distro>-rclcpp ros-<ros2-distro>-sensor-msgs ros-<ros2-distro>-geometry-msgs ros-<ros2-distro>-tf2-msgs
```

Note: `dv-processing` is not in standard apt for most distros — install it according to the library's instructions (source or package).

## Building

From your workspace root (where this package lives):

```bash
source /opt/ros/<ros2-distro>/setup.bash
colcon build --packages-select dv_processing_driver
source install/setup.bash
```

## Running the node

Start with the included launch file (preferred):

```bash
ros2 launch dv_processing_driver capture.launch.py
```

Or run the node executable directly:

```bash
ros2 run dv_processing_driver dv_processor_node
```

## Parameters

You can set these parameters via the ROS 2 parameter mechanism or `--ros-args -p name:=value` on the command line.

- **calibration_path**: path to a calibration file to load (default: empty)
- **accumulate_frames**: enable frame accumulation (default: `true`)
- **imu_frame_name**: IMU frame id (default: `imu_link`)
- **camera_frame_name**: camera frame id (default: `camera_link`)
- **transformImuToCameraFrame**: rotate IMU vectors into camera frame (default: `false`)
- **unbiasedImuData**: apply saved IMU biases (default: `false`)
- **accumulator_decay_param**: accumulator decay parameter (default: `1.0e+6`)
- **accumulator_event_contribution**: single-event contribution (default: `0.15`)
- **accumulator_decay_function**: decay function name (default: `EXPONENTIAL`)

Example: override a parameter when launching:

```bash
ros2 run dv_processing_driver dv_processor_node --ros-args -p accumulate_frames:=false
```

## Topics

- **/events** (type: `dv_processing_driver/msg/EventArray`) — event stream from the camera
- **/imu** (type: `sensor_msgs/Imu`) — IMU measurements
- **/img_accum** (type: `sensor_msgs/Image`) — accumulated frames built from events
- **/tf** (type: `tf2_msgs/TFMessage`) — IMU-to-camera transform when IMU calibration present

Use `ros2 topic list` and `ros2 topic echo <topic>` to inspect live messages.

## Calibration files

The node looks for an "active" calibration file at:

```
~/.dv_camera/camera_calibration/<cameraName>/active_calibration.json
```

If you have a ready calibration file, you can provide it at startup using the `calibration_path` parameter or copy it into the active path before starting the node. Example:

```bash
ros2 run dv_processing_driver dv_processor_node --ros-args -p calibration_path:=/home/user/my_calib.json
# or copy into place (replace <cameraName> appropriately):
cp /home/user/my_calib.json ~/.dv_camera/camera_calibration/<cameraName>/active_calibration.json
```

The node contains helper methods to generate and save calibrations (`generateActiveCalibrationFile()` and `saveCalibration()`), which are used inside the node codebase. If you need runtime services to trigger these, add or call the corresponding API in your integration.

## Recording a bag file for Kalibr

> Kalibr needs synchronized camera images (`sensor_msgs/Image`), a `sensor_msgs/CameraInfo` topic, and IMU data (`sensor_msgs/Imu`). Confirm the topic names with `ros2 topic list` before recording.

1. Start the node (or launch file) so the image and imu topics are publishing.

2. Record a ros2 bag including the image topic (`/img_accum`), the IMU topic (`/imu`) and the camera info topic (if published). Example:

```bash
# record into directory 'kalibr_bag'
ros2 bag record -o kalibr_bag /img_accum /imu /camera_info
```

3. Verify the recorded bag contains the expected topics:

```bash
ros2 bag info kalibr_bag
```

4. Export or convert the bag contents to the format Kalibr expects (images + camera_info YAML + imu text file). Typical Kalibr workflow:

- Extract images from the bag (e.g. using `ros2 bag` tooling or a small script)
- Create a `camera.yaml` camera definition file describing the camera model and intrinsics
- Convert IMU messages into a plain CSV/text file containing timestamps, gyro and acc columns
- Run Kalibr with the prepared data following Kalibr docs

Note: the node publishes an accumulated image topic rather than a raw frame stream; ensure the `img_accum` content and rate meet Kalibr's requirements (sufficient motion and image variety).

### Converting ROS 2 bags to ROS 1 with `rosbags-convert`

A convenient and reliable option for converting ROS 2 bag data into ROS 1 `rosbag` format is the `rosbags` toolkit (which provides a convert command), or a similarly named `rosbags-convert` utility if available on your system.

Install (example using pip):

```bash
pip install rosbags
```

# Using `rosbags-convert`:
```bash
rosbags-convert --src <input_ros2_bag> --dst <output_ros1_bag>
```

After conversion, verify the resulting bag with ROS 1 tools:

```bash
rosbag info kalibr_ros1.bag
```

Notes:

- Confirm that the conversion tool you have supports the specific ROS 2 serialization used by your bag (most modern `rosbags` versions do).
- If messages are custom (non-standard) types, ensure the corresponding message definitions are available to the conversion tool or install matching message packages in the target ROS 1 environment.
- If conversion is not possible for certain topics, another option is to extract images and IMU data directly from the ROS 2 bag and create Kalibr-ready artifacts (images + `camera_info` YAML + IMU CSV).



## Troubleshooting

- If no camera is found, ensure `dv-processing` can open your DVXplorer device (check udev/libusb permissions).
- If no calibration is loaded, check the active calibration path and the `calibration_path` parameter.
- Use `ros2 topic hz <topic>` to check publishing rates and `ros2 topic echo` to inspect messages.

## Development notes

- Executable target: `dv_processor_node` (installed to `lib/dv_processing_driver`)
- Launch file: `launch/capture.launch.py`
- Message definitions are in `msg/Event.msg` and `msg/EventArray.msg`.

