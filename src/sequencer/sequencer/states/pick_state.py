from smach import State
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from interfaces.srv import MoveToPose
from control_msgs.action import GripperCommand
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from gripper.utils.config_loader import GripperConfig
import math
import time
from scipy.spatial.transform import Rotation as R

class PickState(State):
    """State: Execute pick action using individual services"""
    
    def __init__(self, node, release_after_pick=True):
        State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['world_coords', 'orientation', 'grasp_pose', 'pre_grasp_pose']
        )
        self.node = node
        self.release_after_pick = release_after_pick  # Whether to open gripper after pick
        self.move_to_pose_client = self.node.create_client(MoveToPose, '/move_to_pose')
        self.gripper_action_client = ActionClient(self.node, GripperCommand, 'gripper_controller/gripper_cmd')
        config = GripperConfig()
        self.open_position_rad = 0.0
        self.close_position_rad = config.max_position_rad * 0.98
    
    def execute(self, userdata):
        """Execute pick sequence: open gripper -> pre-grasp -> grasp -> close -> retract"""
        
        self.node.get_logger().info('PickState: Starting pick sequence')

        if not self.move_to_pose_client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error('Move to pose service not available')
            return 'failure'
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Gripper action server not available')
            return 'failure'
        
        if not self._send_gripper_goal(open_gripper=True):
            return 'failure'
        
        self.node.get_logger().info('PickState: Moving to pre-grasp pose')
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = 'ur10_base'
        if hasattr(userdata, 'pre_grasp_pose') and userdata.pre_grasp_pose.position.z != 0.0:
            pre_grasp_pose.pose = userdata.pre_grasp_pose
        else:
            pre_grasp_height = 0.1
            orientation_quat = self._compute_orientation(userdata.orientation)
            pre_grasp_pose.pose.position.x = userdata.world_coords.x
            pre_grasp_pose.pose.position.y = userdata.world_coords.y
            pre_grasp_pose.pose.position.z = userdata.world_coords.z + pre_grasp_height
            pre_grasp_pose.pose.orientation = orientation_quat
        
        move_request = MoveToPose.Request()
        move_request.target_pose = pre_grasp_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to move to pre-grasp pose')
            return 'failure'
        

        self.node.get_logger().info('PickState: Moving to grasp pose')
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'ur10_base'
        if hasattr(userdata, 'grasp_pose') and userdata.grasp_pose.position.z != 0.0:
            grasp_pose.pose = userdata.grasp_pose
        else:
            orientation_quat = self._compute_orientation(userdata.orientation)
            grasp_pose.pose.position.x = userdata.world_coords.x
            grasp_pose.pose.position.y = userdata.world_coords.y
            grasp_pose.pose.position.z = userdata.world_coords.z
            grasp_pose.pose.orientation = orientation_quat
        
        move_request = MoveToPose.Request()
        move_request.target_pose = grasp_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to move to grasp pose')
            return 'failure'
        
        if not self._send_gripper_goal(open_gripper=False):
            return 'failure'

        self.node.get_logger().info('PickState: Retracting to pre-grasp')
        move_request = MoveToPose.Request()
        move_request.target_pose = pre_grasp_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to retract')
            return 'failure'
        
        if self.release_after_pick:
            if not self._send_gripper_goal(open_gripper=True, warn_only=True):
                self.node.get_logger().warn('Failed to open gripper, but pick completed')
        
        self.node.get_logger().info('PickState: Pick sequence completed successfully!')
        return 'success'

    def _send_gripper_goal(self, open_gripper: bool, warn_only: bool = False) -> bool:
        """Send a GripperCommand goal via MoveIt action interface."""
        goal = GripperCommand.Goal()
        desired = self.open_position_rad if open_gripper else self.close_position_rad
        goal.command.position = desired
        goal.command.max_effort = 20.0  # mNm, adjust as needed

        future = self.gripper_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if not future.done():
            msg = 'Gripper goal not accepted'
            if warn_only:
                self.node.get_logger().warn(msg)
                return False
            self.node.get_logger().error(msg)
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            msg = 'Gripper action goal rejected'
            if warn_only:
                self.node.get_logger().warn(msg)
                return False
            self.node.get_logger().error(msg)
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            msg = f'Gripper action failed with status {result.status}'
            if warn_only:
                self.node.get_logger().warn(msg)
                return False
            self.node.get_logger().error(msg)
            return False

        self.node.get_logger().info(
            f'Gripper action succeeded (position={goal.command.position:.3f})')
        return True

    def _compute_orientation(self, orientation_deg: float) -> Quaternion:
        orientation_rad = math.radians(orientation_deg)
        base_rotation = R.from_euler('y', math.pi)
        z_rotation = R.from_euler('z', orientation_rad)
        quat = (z_rotation * base_rotation).as_quat()
        orientation_quat = Quaternion()
        orientation_quat.x = quat[0]
        orientation_quat.y = quat[1]
        orientation_quat.z = quat[2]
        orientation_quat.w = quat[3]
        return orientation_quat

