# RSN Handover Demo

ROS 2 Python package for a voice-guided surgical instrument handover demo.

The current pipeline is:

1. `voice_command_node` listens for a spoken instrument request and publishes a target class.
2. `instrument_detection_node` detects the requested instrument with YOLO and ZED, publishes one grasp pose, then exits to release the camera.
3. `xarm_controller_node` moves the xArm to grasp and lift the instrument.
4. `demo_coordinator` starts `zed_hand_node` after the instrument detector exits.
5. `zed_hand_node` detects the handover point with MediaPipe and ZED, then publishes one hand pose.
6. `xarm_controller_node` moves to the handover point and uses the force-torque sensor to detect release.

## Environment

Expected base environment:

- ROS 2 Humble
- Python 3.10
- xArm reachable at the IP configured in `config/xarm_controller_params.yaml`
- ZED SDK and Python API available as `pyzed.sl`
- YOLO weights path configured in `config/instrument_detection_params.yaml`

Python/runtime dependencies used by the nodes:

- `rclpy`
- `geometry_msgs`
- `std_msgs`
- `std_srvs`
- `numpy`
- `opencv-python` / `cv2`
- `ultralytics`
- `mediapipe`
- `sounddevice`
- `soundfile`
- `SpeechRecognition`
- `xarm-python-sdk`
- ZED Python API (`pyzed`)

Install the pip-managed Python dependencies with:

```bash
python3 -m pip install -r src/rsn/requirements.txt
```

`pyzed` is provided by the ZED SDK and is not listed in `requirements.txt`.

## Build

From the workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select rsn
source install/setup.bash
```

## Run

```bash
ros2 launch rsn handover_demo.launch.py
```

The launch file loads all runtime parameters from `config/*.yaml` through the installed package share directory. Rebuild the package after editing config files if you run from the installed workspace.

## Configuration

- `config/xarm_controller_params.yaml`: robot IP, fixed poses, gripper values, hand/instrument approach offsets, release detection thresholds.
- `config/instrument_detection_params.yaml`: YOLO model path, ZED serial, detection thresholds, mask processing, grasp-plane calibration offsets.
- `config/zed_hand_params.yaml`: ZED hand detection, MediaPipe settings, depth window, hand pose topic.
- `config/voice_command_params.yaml`: microphone recording parameters, speech topics, debug behavior.
- `config/demo_coordinator_params.yaml`: demo sequencing delays, retry counts, and automatic hand-node launch settings.

## ROS Interfaces

Published topics:

- `/voice_target_instrument` (`std_msgs/String`): target YOLO class selected from speech.
- `/voice_recognized_text` (`std_msgs/String`): raw speech recognition result when enabled.
- `/instrument_grasp_pose_base` (`geometry_msgs/PoseStamped`): instrument grasp pose in `xarm_base`.
- `/right_hand_pose_base` (`geometry_msgs/PoseStamped`): handover pose in `xarm_base`.

Subscribed topics:

- `instrument_detection_node` subscribes to `/voice_target_instrument`.
- `xarm_controller_node` subscribes to `/instrument_grasp_pose_base` and `/right_hand_pose_base`.
- `demo_coordinator` subscribes to `/voice_target_instrument`.

Services:

- `/start_instrument_detection` (`std_srvs/Trigger`)
- `/start_hand_detection` (`std_srvs/Trigger`)
- `/move_to_p0` (`std_srvs/Trigger`)
- `/move_to_p1` (`std_srvs/Trigger`)
- `/move_to_instrument` (`std_srvs/Trigger`)
- `/lift_after_grasp` (`std_srvs/Trigger`)
- `/move_to_hand` (`std_srvs/Trigger`)
- `/wait_for_release` (`std_srvs/Trigger`)
- `/retreat_after_release` (`std_srvs/Trigger`)
- `/open_gripper` (`std_srvs/Trigger`)
- `/close_gripper` (`std_srvs/Trigger`)

## Notes For FlexBE Integration

The current `demo_coordinator` owns the full procedural demo sequence. For FlexBE integration, treat perception, robot motion, gripper control, and release detection as lower-level primitives, then move sequencing logic out of `demo_coordinator` and into a behavior/state machine.

Good candidates for future Actions:

- Instrument detection: goal is target class, result is grasp pose.
- Hand detection: result is handover pose.
- Wait for release: feedback is current force/hold time, result is released or timeout.
- Arm movement: goal is named pose or target pose, result is motion status.

The instrument detector exits after publishing because the same ZED camera is reused by the hand detector.
