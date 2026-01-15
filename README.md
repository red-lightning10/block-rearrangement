# Manipulation-Net Benchmarking - Block Rearrangement - ELPIS Lab

## Hardware

- **Robot**: UR10 (IP: 192.168.0.100)
- **Gripper**: Custom gripper with librh_p12_rna.so
- **Camera**: Intel RealSense D435

## Packages

- **`interfaces`** - ROS2 service/action definitions
- **`grasping`** - Robot movement, gripper control, grasp execution
- **`world_coords`** - Get location of cube centers in 3D
- **`segmentation`** - Cube detection and segmentation
- **`sequencer`** - SMACH state machine to perform input sequence of actions
- **`realsense-ros`** - RealSense camera ROS2 package

## Notes

- Robot IP and HOME_CONFIG can be configured in `robot_utils.py`
