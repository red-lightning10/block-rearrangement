from sensor_msgs.msg import JointState
from gripper.utils.config_loader import GripperConfig

_config = GripperConfig()

def raw_to_radians(position_raw: float) -> float:
    """
    Convert raw gripper position to radians
    
    Args:
        position_raw: Raw position value (0 to max_position)
    
    Returns:
        float: Position in radians (0 to max_position_rad)
    """
    if position_raw < 0:
        return 0.0
    return (position_raw / _config.max_position) * _config.max_position_rad


def radians_to_raw(position_rad: float) -> int:
    """
    Convert radians to raw gripper position
    
    Args:
        position_rad: Position in radians (0 to max_position_rad)
    
    Returns:
        int: Raw position value (0 to max_position)
    """
    if position_rad < 0:
        return 0
    if position_rad > _config.max_position_rad:
        return _config.max_position
    return int((position_rad / _config.max_position_rad) * _config.max_position)


def create_gripper_joint_state(position_rad: float, effort: float = 0.0, timestamp=None) -> JointState:
    """
    Create a JointState message for gripper joints
    
    Args:
        position_rad: Gripper position in radians
        timestamp: Optional timestamp (if None, will need to be set by caller)
    
    Returns:
        JointState: Joint state message with all gripper joints
    """
    joint_state = JointState()
    if timestamp is not None:
        joint_state.header.stamp = timestamp
    joint_state.name = _config.gripper_joint_names
    joint_state.position = [position_rad] * len(_config.gripper_joint_names)
    joint_state.velocity = [0.0] * len(_config.gripper_joint_names)
    joint_state.effort = [effort] * len(_config.gripper_joint_names)
    return joint_state


def merge_joint_states(ur_joint_state: JointState, gripper_joint_state: JointState = None) -> JointState:
    """
    Merge UR arm and gripper joint states into a single message
    
    Args:
        ur_joint_state: Joint state from UR arm
        gripper_joint_state: Optional joint state from gripper
    
    Returns:
        JointState: Merged joint state message
    """
    merged = JointState()
    merged.header = ur_joint_state.header
    
    # Start with UR joints
    merged.name = list(ur_joint_state.name)
    merged.position = list(ur_joint_state.position)
    merged.velocity = list(ur_joint_state.velocity)
    merged.effort = list(ur_joint_state.effort) if ur_joint_state.effort else []
    
    # Add or overwrite gripper joints if available
    if gripper_joint_state is not None:
        for i, joint_name in enumerate(gripper_joint_state.name):
            position = gripper_joint_state.position[i]
            velocity = gripper_joint_state.velocity[i] if i < len(gripper_joint_state.velocity) else 0.0
            
            if gripper_joint_state.effort and i < len(gripper_joint_state.effort):
                effort = gripper_joint_state.effort[i]
            else:
                effort = 0.0

            if joint_name in merged.name:
                idx = merged.name.index(joint_name)
                merged.position[idx] = position

                if idx < len(merged.velocity):
                    merged.velocity[idx] = velocity
                else:
                    merged.velocity.append(velocity)

                if merged.effort:
                    if idx < len(merged.effort):
                        merged.effort[idx] = effort
                    else:
                        merged.effort.append(effort)
                else:
                    merged.effort = [0.0] * len(merged.position)
                    merged.effort[idx] = effort
            else:
                merged.name.append(joint_name)
                merged.position.append(position)
                merged.velocity.append(velocity)
                merged.effort.append(effort)
    
    return merged


