# Gripper Package

ROS 2 package for controlling the Robotis RH-P12-RNA gripper using `librh_p12_rna.so`.

## Overview

This package provides:
- **Action Server**: MoveIt-compatible `control_msgs/action/GripperCommand` action server for gripper control
- **Service Server**: Simple service-based control interface (`/control_gripper`)
- **Joint State Merger**: Merges UR arm and gripper joint states for robot state publisher

## Package Structure

```
gripper/
├── gripper/
│   ├── gripper_action_server.py    # MoveIt-compatible action server
│   ├── control_gripper_server.py   # Service-based control server
│   ├── joint_state_merger.py       # Merges UR + gripper joint states
│   └── utils/
│       ├── gripper_controller.py   # Low-level hardware interface
│       └── joint_state_utils.py    # Joint state conversion utilities
├── launch/
│   └── gripper.launch.py           # Launch file for action server + merger
└── lib/
    └── librh_p12_rna.so            # Proprietary gripper library
```

## Usage

### Launch with Robot

The gripper is typically launched as part of the robot system:

```bash
ros2 launch ur_elpis_control start_robot.launch.py
```

This automatically starts:
- `gripper_action_server` (for MoveIt integration)
- `joint_state_merger` (for joint state publishing)

### Standalone Launch

To launch just the gripper components:

```bash
ros2 launch gripper gripper.launch.py
```

### Service-Based Control

If you need the service interface (non-MoveIt use cases):

```bash
ros2 run gripper control_gripper_server
```

Then use the service:
```bash
# Open gripper
ros2 service call /control_gripper interfaces/srv/ControlGripper "{command: 'open'}"

# Close gripper
ros2 service call /control_gripper interfaces/srv/ControlGripper "{command: 'close'}"

# Set specific position (0-740)
ros2 service call /control_gripper interfaces/srv/ControlGripper "{command: 'position:370'}"
```

## Configuration

### Hardcoded Gripper Links

**⚠️ IMPORTANT**: The gripper joint names are hardcoded in the package. If you need to change them, modify:

**File**: `gripper/utils/joint_state_utils.py`

**Line 8**: 
```python
GRIPPER_JOINT_NAMES = ['rh_p12_rn', 'rh_r2', 'rh_l1', 'rh_l2']
```

This constant is used by:
- `joint_state_merger.py` - to identify gripper joints in joint state messages
- `joint_state_utils.py` - to create gripper joint state messages

### Position Limits

Position conversion constants are also in `joint_state_utils.py`:
- `MAX_POSITION_RAW = 740` - Maximum raw position value (fully closed)
- `MAX_POSITION_RAD = 1.12` - Maximum position in radians (fully closed)

### Library Path

The gripper library path is hardcoded in:

**File**: `gripper/utils/gripper_controller.py`

**Lines 12-14**:
```python
LIB_PATH = os.path.abspath(os.path.join(get_package_share_directory('gripper'), \
                                         '..', '..', 'lib', 'gripper', \
                                         'librh_p12_rna.so'))
```

This resolves to: `install/gripper/lib/gripper/librh_p12_rna.so`

## MoveIt Integration

The action server listens on the namespace MoveIt expects:
- **Action**: `/gripper_controller/gripper_cmd`
- **Type**: `control_msgs/action/GripperCommand`

Position values are in radians (0.0 = fully open, 1.12 = fully closed).

## Dependencies

- `rclpy` - ROS 2 Python client library
- `control_msgs` - For `GripperCommand` action
- `sensor_msgs` - For `JointState` messages
- `interfaces` - Custom service definitions
- `ament_index_python` - For package path resolution

## Notes

- Only one process can control the gripper at a time (single port access)
- The action server publishes joint states to `/joint_states` directly
- The joint state merger subscribes to `/gripper_joint_states` and merges with UR arm states
- Position control mode 5 is used for stable gripper holding

For more detailed development notes, see [NOTES.md](NOTES.md).


