from smach import State
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from interfaces.srv import MoveToPose
import math
from scipy.spatial.transform import Rotation as R

HOME_X = 0.1215
HOME_Y = -0.745
HOME_Z = 0.32325
HOME_RX = 0.0
HOME_RY = -math.pi
HOME_RZ = 0.0

class HomeState(State):
    """State: Move robot to home position"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'failure']
        )
        self.node = node
        self.client = self.node.create_client(MoveToPose, '/move_to_pose')

        self.home = [HOME_X, HOME_Y, HOME_Z, HOME_RX, HOME_RY, HOME_RZ]
    
    def execute(self, userdata):
        """Execute move to home"""
        
        self.node.get_logger().info('HomeState: Moving to home position')
        
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=30.0):
            self.node.get_logger().error('Move to pose service not available after 30 seconds. Is move_to_pose_server running?')
            return 'failure'
        
        # Create home pose quaternion from rotation vector (axis-angle representation)
        rotation = R.from_rotvec([self.home[3], self.home[4], self.home[5]])
        quat = rotation.as_quat()
        home_orientation = Quaternion()
        home_orientation.x = quat[0]
        home_orientation.y = quat[1]
        home_orientation.z = quat[2]
        home_orientation.w = quat[3]
        
        home_pose = PoseStamped()
        home_pose.header.frame_id = 'ur10_base'
        home_pose.pose.position.x = self.home[0]
        home_pose.pose.position.y = self.home[1]
        home_pose.pose.position.z = self.home[2]
        home_pose.pose.orientation = home_orientation
        
        request = MoveToPose.Request()
        request.target_pose = home_pose
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=60.0)
        
        if not future.done():
            self.node.get_logger().error('Move to home service call timed out')
            return 'failure'
        
        response = future.result()
        
        if response.success:
            self.node.get_logger().info(f'HomeState: {response.message}')
            return 'success'
        else:
            self.node.get_logger().error(f'HomeState failed: {response.message}')
            return 'failure'

