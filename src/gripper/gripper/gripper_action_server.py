import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from gripper.utils.gripper_controller import GripperController
from gripper.utils.joint_state_utils import raw_to_radians, radians_to_raw, create_gripper_joint_state
from gripper.utils.config_loader import GripperConfig
import time


class GripperActionServer(Node):
    """Action server for gripper control compatible with MoveIt"""
    
    def __init__(self, config: GripperConfig = None):
        super().__init__('gripper_action_server')
        
        self.config = config if config is not None else GripperConfig()
        self.gripper = GripperController(self, self.config)
        
        # Create action server on the namespace MoveIt expects
        self.action_server = ActionServer(self, GripperCommand, self.config.gripper_command_action, self.execute_callback)

        self.joint_state_pub = self.create_publisher(JointState, self.config.gripper_joint_states_topic, 
                                                    self.config.queue_size)
        publish_period = 1.0 / self.config.publish_rate
        self.joint_state_timer = self.create_timer(publish_period, self.publish_gripper_joint_states)

        # Service to query filtered holding state (median of boolean samples)
        self.holding_service = self.create_service(Trigger, self.config.is_holding_service, 
                                                    self.handle_is_holding_request)
        self.get_logger().info(f'Gripper Action Server started on: {self.config.gripper_command_action}')
    
    def publish_gripper_joint_states(self) -> None:
        """Publish gripper joint states - subscribed by joint_state_merger to merge with UR arm states"""
        if self.gripper.lib is None:
            return
        
        try:
            position_raw = self.gripper.get_position()
            if position_raw < 0:
                return

            current_ma = max(0.0, self.gripper.get_current_ma())
            position_rad = raw_to_radians(position_raw)
            
            joint_state = create_gripper_joint_state(position_rad, effort=current_ma, 
                                                    timestamp=self.get_clock().now().to_msg())
            self.joint_state_pub.publish(joint_state)
        
        except Exception as e:
            self.get_logger().warn(f'Failed to publish gripper joint states: {e}') 

    def handle_is_holding_request(self, _request, response) -> Trigger.Response:
        """Handle is_holding service request - samples gripper state multiple times to eliminate noise"""
        sample_count = self.config.holding_sample_count
        is_holding = sum(self.gripper.is_gripping() for _ in range(sample_count)) >= (sample_count / 2)
        
        response.success = is_holding
        response.message = 'Gripper holding object' if is_holding else 'Gripper empty'
        return response

    def _create_result(self, position: float, reached_goal: bool, stalled: bool) -> GripperCommand.Result:
        """Helper to create GripperCommand.Result with consistent structure"""
        result = GripperCommand.Result()
        result.position = position
        result.reached_goal = reached_goal
        result.stalled = stalled
        return result
    
    def _is_goal_reached(self, current_rad: float, target_rad: float, current_raw: int, target_raw: int) -> bool:
        """Check if gripper has reached the goal position"""
        error_rad = abs(current_rad - target_rad)
        error_raw = abs(current_raw - target_raw)
        return error_rad < self.config.position_tolerance_rad or error_raw <= self.config.position_tolerance_raw
    
    def execute_callback(self, goal_handle):
        """Execute gripper command action"""
        goal = goal_handle.request.command.position
        self.get_logger().info(f'Received gripper command: position={goal} meters')
        
        # Convert from meters/radians to raw position
        target_rad = float(goal)
        target_raw = radians_to_raw(target_rad)
        self.get_logger().info(f'Target: {target_rad} rad -> {target_raw} raw')
        
        # Check if gripper is initialized
        if self.gripper.lib is None:
            self.get_logger().error('Gripper library not loaded')
            goal_handle.abort()
            return self._create_result(raw_to_radians(self.gripper.get_position()), False, False)
        
        # Get initial position
        initial_raw = self.gripper.get_position()
        if initial_raw < 0:
            self.get_logger().error('Failed to read initial gripper position')
            goal_handle.abort()
            return self._create_result(0.0, False, False)
        
        initial_rad = raw_to_radians(initial_raw)
        self.get_logger().info(f'Initial position: {initial_rad} rad ({initial_raw} raw)')
        
        # Check if already at goal
        if self._is_goal_reached(initial_rad, target_rad, initial_raw, target_raw):
            self.get_logger().info('Already at target position')
            goal_handle.succeed()
            return self._create_result(initial_rad, True, False)
        
        # Send command to gripper
        if not self.gripper.set_position(target_raw):
            self.get_logger().error('Failed to send position command to gripper')
            goal_handle.abort()
            return self._create_result(initial_rad, False, False)
        
        # Monitor movement until goal is reached or timeout
        start_time = time.time()
        last_position = initial_raw
        stalled_count = 0
        feedback_period = 1.0 / self.config.feedback_rate
        
        while True:
            # Check timeout first (before reading position to avoid unnecessary I/O)
            elapsed = time.time() - start_time
            if elapsed > self.config.movement_timeout:
                self.get_logger().warn(f'Gripper movement timed out after {elapsed:.2f}s')
                # Read final position and check if we actually reached goal or are gripping
                final_raw = self.gripper.get_position()
                final_rad = raw_to_radians(final_raw) if final_raw >= 0 else initial_rad
                
                if self._is_goal_reached(final_rad, target_rad, final_raw, target_raw):
                    goal_handle.succeed()
                    return self._create_result(final_rad, True, False)
                
                # Check if gripping despite timeout
                is_gripping = self.gripper.is_gripping()
                if is_gripping:
                    self.get_logger().info('Timeout but gripping object - SUCCESS')
                    goal_handle.succeed()
                    return self._create_result(final_rad, True, True)
                else:
                    self.get_logger().warn('Timeout without gripping - FAILURE')
                    goal_handle.abort()
                    return self._create_result(final_rad, False, False)
            
            current_raw = self.gripper.get_position()
            if current_raw < 0:
                self.get_logger().warn('Failed to read position during movement')
                time.sleep(feedback_period)
                continue
            
            current_rad = raw_to_radians(current_raw)
            
            # Publish feedback
            feedback = GripperCommand.Feedback()
            feedback.position = current_rad
            goal_handle.publish_feedback(feedback)
            
            # Check if goal reached
            if self._is_goal_reached(current_rad, target_rad, current_raw, target_raw):
                self.get_logger().info(
                    f'Goal reached! Position: {current_rad:.4f} rad ({current_raw:.0f} raw) '
                    f'(target: {target_rad:.4f} rad / {target_raw} raw)')
                goal_handle.succeed()
                return self._create_result(current_rad, True, False)
            
            # Check for stalling (position not changing)
            if abs(current_raw - last_position) < self.config.stall_detection_threshold:
                stalled_count += 1
                if stalled_count > self.config.stall_count_limit:
                    is_gripping = self.gripper.is_gripping()
                    if is_gripping:
                        self.get_logger().info('Gripper stalled while gripping object - SUCCESS')
                        goal_handle.succeed()
                        return self._create_result(current_rad, True, True)
                    else:
                        self.get_logger().warn('Gripper stalled without gripping - FAILURE')
                        goal_handle.abort()
                        return self._create_result(current_rad, False, True)
            else:
                stalled_count = 0
            
            last_position = current_raw
            time.sleep(feedback_period)

def main(args=None):
    rclpy.init(args=args)
    
    server = GripperActionServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.gripper.cleanup()
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

