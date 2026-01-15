from smach import State
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from interfaces.srv import MoveToPose
from control_msgs.action import GripperCommand
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from gripper.utils.config_loader import GripperConfig
import math
from scipy.spatial.transform import Rotation as R

class PlaceState(State):
    """State: Move to drop location and release object"""
    
    def __init__(self, node, drop_location=None):
        State.__init__(self, outcomes=['success', 'failure'])
        self.node = node
        if drop_location is None:
            self.node.get_logger().error('PlaceState: drop_location must be provided')
            return
        else:
            self.drop_location = drop_location
            self.move_to_pose_client = self.node.create_client(MoveToPose, '/move_to_pose')
            self.gripper_action_client = ActionClient(self.node, GripperCommand, 'gripper_controller/gripper_cmd')
            config = GripperConfig()
            self.open_position_rad = 0.0
            self.close_position_rad = config.max_position_rad * 0.98

    def execute(self, userdata):
        """Execute place sequence (placeholder): move to drop location -> open gripper -> close gripper -> retract"""
        
        self.node.get_logger().info('PlaceState: Moving to drop location')
        
        # Wait for services
        if not self.move_to_pose_client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error('Move to pose service not available')
            return 'failure'
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Gripper action server not available')
            return 'failure'
        
        rotation = R.from_rotvec([0.0, -math.pi, 0.0])
        quat = rotation.as_quat()
        home_orientation = Quaternion()
        home_orientation.x = quat[0]
        home_orientation.y = quat[1]
        home_orientation.z = quat[2]
        home_orientation.w = quat[3]
     
        self.node.get_logger().info('PlaceState: Moving to drop location')
        drop_pose = PoseStamped()
        drop_pose.header.frame_id = 'ur10_base'
        drop_pose.pose.position.x = self.drop_location['x']
        drop_pose.pose.position.y = self.drop_location['y']
        drop_pose.pose.position.z = self.drop_location['z']
        drop_pose.pose.orientation = home_orientation
        
        move_request = MoveToPose.Request()
        move_request.target_pose = drop_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to move to drop location')
            return 'failure'
        
        if not self._send_gripper_goal(open_gripper=True):
            return 'failure'
        
        if not self._send_gripper_goal(open_gripper=False, warn_only=True):
            self.node.get_logger().warn('Failed to close gripper, but placement completed')

        self.node.get_logger().info('PlaceState: Retracting up')
        move_request = MoveToPose.Request()
        move_request.target_pose = drop_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().warn('Failed to retract, but placement completed')
        
        self.node.get_logger().info('PlaceState: Successfully placed and released object')
        return 'success'

    def _send_gripper_goal(self, open_gripper: bool, warn_only: bool = False) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = self.open_position_rad if open_gripper else self.close_position_rad
        goal.command.max_effort = 20.0

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

