# FlexBE Migration Plan

This document records the migration path from the original `demo_coordinator`
procedural sequence to FlexBE-controlled task execution.

## Goal

Keep the working handover demo stable while moving high-level sequencing into
FlexBE. The first FlexBE version should reuse the existing ROS interfaces:

- `std_srvs/Trigger` services for robot and perception commands.
- Topics for voice target, instrument pose, and hand pose.
- Existing launch/config files for all low-level nodes.

Do not convert services to actions in the first FlexBE pass. Add actions later
only where feedback, cancellation, or explicit result data is actually needed.

## Current Runtime Ownership

The original `demo_coordinator` owns the demo sequence:

1. Move xArm to P0.
2. Open gripper.
3. Wait for voice target on `/voice_target_instrument`.
4. Start instrument detection.
5. Move to the detected instrument pose.
6. Close gripper.
7. Lift after grasp.
8. Wait for `instrument_detection_node` to release the ZED camera.
9. Launch `zed_hand_node`.
10. Start hand detection.
11. Move to detected hand pose.
12. Wait for force-based release.
13. Open gripper.
14. Retreat.
15. Return to P0.

The first FlexBE migration is now complete: `RSN Handover Demo` owns this
sequence, and the existing nodes remain low-level providers of perception,
motion, gripper, and release-detection primitives. `demo_coordinator` remains
as a legacy fallback runner.

## Proposed Package Layout

Create a separate FlexBE package instead of mixing generated behavior code into
`rsn`:

```text
ros2_ws/src/rsn_flexbe_behaviors/
```

Keep `rsn` as the runtime node package. Keep FlexBE states and behaviors in the
new package.

Current structure:

```text
rsn_flexbe_behaviors/
  package.xml
  setup.py
  setup.cfg
  resource/rsn_flexbe_behaviors
  manifest/
    handover_demo.xml
  rsn_flexbe_behaviors/
    __init__.py
    handover_demo_sm.py
    states/
      __init__.py
      trigger_service_state.py
      wait_for_voice_target_state.py
      launch_hand_node_state.py
```

## Initial State Mapping

| FlexBE state | Existing interface | Notes |
| --- | --- | --- |
| `MoveToP0State` | `/move_to_p0` (`std_srvs/Trigger`) | Return `done` or `failed`. |
| `OpenGripperState` | `/open_gripper` (`std_srvs/Trigger`) | Used at start and release. |
| `WaitForVoiceTargetState` | `/voice_target_instrument` (`std_msgs/String`) | Store target string in userdata if needed. |
| `StartInstrumentDetectionState` | `/start_instrument_detection` (`std_srvs/Trigger`) | Requires voice target to have already arrived. |
| `MoveToInstrumentState` | `/move_to_instrument` (`std_srvs/Trigger`) | Existing xArm node uses latest instrument pose cache. |
| `CloseGripperState` | `/close_gripper` (`std_srvs/Trigger`) | Grasp command. |
| `LiftAfterGraspState` | `/lift_after_grasp` (`std_srvs/Trigger`) | Lift after successful grasp. |
| `LaunchHandNodeState` | `ros2 run rsn zed_hand_node --ros-args --params-file ...` | First version can mirror `demo_coordinator` subprocess logic. |
| `StartHandDetectionState` | `/start_hand_detection` (`std_srvs/Trigger`) | Wait for service after hand node launch. |
| `MoveToHandState` | `/move_to_hand` (`std_srvs/Trigger`) | Existing xArm node uses latest hand pose cache. |
| `WaitForReleaseState` | `/wait_for_release` (`std_srvs/Trigger`) | Blocking service call is acceptable for first version. |
| `OpenGripperForReleaseState` | `/open_gripper` (`std_srvs/Trigger`) | Same service as start open. |
| `RetreatAfterReleaseState` | `/retreat_after_release` (`std_srvs/Trigger`) | Retreat after handover. |
| `ReturnToP0State` | `/move_to_p0` (`std_srvs/Trigger`) | Final return. |

Most of these states can be implemented by one reusable
`TriggerServiceState`.

## Reusable States

### TriggerServiceState

Purpose: call a `std_srvs/Trigger` service and return a FlexBE outcome.

Inputs:

- `service_name`
- `timeout_sec`
- optional `retry_count`
- optional `retry_delay_sec`

Outcomes:

- `done`: service replied with `success=True`.
- `failed`: service replied with `success=False`.
- `unavailable`: service did not become available before timeout.

### WaitForVoiceTargetState

Purpose: wait until a target class is published by `voice_command_node`.

Inputs:

- `topic`, default `/voice_target_instrument`
- `timeout_sec`

Outputs:

- `target_class`

Outcomes:

- `received`
- `timeout`

### LaunchHandNodeState

Purpose: start `zed_hand_node` only when the behavior reaches the hand-detection
phase, so the ZED camera is not opened by both perception nodes at the same
time.

Inputs:

- `cmd`
- `startup_delay_sec`
- optional `required_service_name`
- optional `service_timeout_sec`

Outcomes:

- `launched`
- `failed`
- `service_unavailable`

The first version is specific to launching `zed_hand_node`. Later, it should be
replaced by a generic `LaunchProcessState`, a ROS launch-based approach, or a
lifecycle-node transition.

## Behavior Graph

Implemented first behavior graph:

```text
MoveToP0
  -> OpenGripper
  -> WaitForVoiceTarget
  -> StartInstrumentDetection
  -> MoveToInstrument
  -> CloseGripper
  -> LiftAfterGrasp
  -> LaunchHandNode
  -> StartHandDetection
  -> MoveToHand
  -> WaitForRelease
  -> OpenGripperForRelease
  -> RetreatAfterRelease
  -> ReturnToP0
  -> finished
```

Every service failure currently transitions to a common `failed` outcome. The
first behavior is intentionally linear because the original demo is linear and
already validated.

## Launch Strategy

For the first FlexBE test:

1. Launch low-level nodes without `demo_coordinator`.
2. Start FlexBE separately.
3. Run the FlexBE behavior to drive the sequence.

The low-level launch should include:

- `xarm_controller_node`
- `instrument_detection_node`
- `voice_command_node`

It should not start:

- `demo_coordinator`
- `zed_hand_node` at startup

The FlexBE behavior should start `zed_hand_node` only after
`instrument_detection_node` publishes once and exits.

Implemented launch entries:

- `rsn/launch/handover_flexbe_low_level.launch.py`: starts
  `xarm_controller_node`, `instrument_detection_node`, and
  `voice_command_node`.

## What To Do With demo_coordinator

Keep `demo_coordinator` as a proven reference implementation and fallback demo
runner. It is now marked as the legacy procedural entry point in the README.
Do not delete it until the FlexBE behavior has recovery logic and enough test
coverage to replace it completely.

## Phase 1 Completion Status

Completed:

1. Created `rsn_flexbe_behaviors`.
2. Implemented `TriggerServiceState`.
3. Implemented `WaitForVoiceTargetState`.
4. Implemented `LaunchHandNodeState`.
5. Created the linear `RSN Handover Demo` behavior.
6. Added a low-level RSN launch file without `demo_coordinator`.
7. Verified the FlexBE behavior with real hardware.

## Phase 2: Recovery And Robustness

The next step is to make the behavior robust instead of only linear.

Recommended recovery branches:

- `Move To Instrument` failure: retry detection or return to
  `Start Instrument Detection`.
- `Move To Hand` failure: retry hand detection or return to
  `Start Hand Detection`.
- `Wait For Release` timeout: open gripper, retreat, and return to P0 instead
  of ending in a generic failure.
- `Launch Hand Node` failure: retry launch once, then transition to a safe
  failure path.
- Any unrecoverable failure: prefer a safe `Return To P0` path where possible.

Keep these recovery paths in FlexBE. Do not bury task-level recovery inside the
low-level nodes.

## Phase 3: Action Candidates

Convert to ROS actions only after the first FlexBE behavior runs reliably.

Strong candidates:

- Instrument detection: goal is target class, feedback is detection stability,
  result is grasp pose.
- Hand detection: feedback is hand visibility/stability, result is hand pose.
- Wait for release: feedback is force magnitude and hold duration, result is
  released or timeout.
- Arm motion: goal is named pose or target pose, feedback is motion state,
  result is final status.

Keep simple gripper open/close as services unless cancellation or feedback
becomes necessary.

## Next Implementation Steps

1. Create `rsn_flexbe_behaviors`.
2. Implement `TriggerServiceState`.
3. Implement `WaitForVoiceTargetState`.
4. Implement `LaunchHandNodeState`.
5. Create a linear handover behavior using the current validated sequence.
6. Add a launch file that starts low-level RSN nodes without
   `demo_coordinator`.
7. Test the FlexBE behavior with the real robot only after dry service calls
   and topic subscriptions work.
