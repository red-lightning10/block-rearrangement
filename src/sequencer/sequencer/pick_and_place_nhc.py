import rclpy
from rclpy.node import Node
import smach
from smach import State
from sequencer.states import HomeState, ProjectState, PickState
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from interfaces.srv import MoveToPose, GetSegmentation, ProjectTo3D, GetPlacement
from control_msgs.action import GripperCommand
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from gripper.utils.config_loader import GripperConfig
from interfaces.msg import DetectionArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time
from scipy.spatial.transform import Rotation as R
import threading

# Class IDs
CLASS_ID_PICK = 2  # Objects to pick

# Drop offset: 2.5 cm above the placement location
DROP_OFFSET_Z = 0.025  # 2.5 cm in meters
PRE_GRASP_HEIGHT = 0.1  # 10 cm above for pre-grasp position


class FilterPickClassDetectionsState(State):
    """State: Filter detections to only class_id == CLASS_ID_PICK and store them"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'no_objects', 'failure'],
            input_keys=['all_detections'],
            output_keys=['all_detections', 'current_index']
        )
        self.node = node
        self.node = node
        self.segmentation_client = self.node.create_client(GetSegmentation, '/get_segmentation')
    
    def execute(self, userdata):
        """Read detections topic and filter for class_id == CLASS_ID_PICK"""
        
        if not self.segmentation_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('FilterPickClassDetections: Segmentation service unavailable')
            return 'failure'

        try:
            request = GetSegmentation.Request()
            future = self.segmentation_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        except Exception as exc:
            self.node.get_logger().error(f'FilterPickClassDetections: Segmentation call failed: {exc}')
            return 'failure'

        self.node.get_logger().info(
            f'FilterPickClassDetections: Waiting for detections on /detections (class_id={CLASS_ID_PICK})')
        
        detection_msg = self.node.wait_for_detections(timeout_sec=5.0)
        if detection_msg is None:
            self.node.get_logger().warn('FilterPickClassDetections: No new detections from topic within timeout')
            return 'failure'
        
        if detection_msg.success and len(detection_msg.detections) > 0:
            # Clear previous detections
            old_count = len(userdata.all_detections) if userdata.all_detections else 0
            userdata.centroid = None
            userdata.depth = None
            userdata.orientation = None
            userdata.world_coords = None
            
            if old_count > 0:
                self.node.get_logger().info(f'FilterPickClassDetections: Clearing {old_count} previous detection(s)')
            
            # Filter for class_id == CLASS_ID_PICK only
            pick_class_detections = [d for d in detection_msg.detections if d.class_id == CLASS_ID_PICK]

            if len(pick_class_detections) == 0:
                self.node.get_logger().info(f'FilterPickClassDetections: No class {CLASS_ID_PICK} objects found')
                return 'no_objects'
            
            # Store filtered detections
            userdata.all_detections = pick_class_detections
            userdata.current_index = 0
            
            self.node.get_logger().info(
                f'FilterPickClassDetections: Found {len(pick_class_detections)} class {CLASS_ID_PICK} object(s) to pick'
            )
            return 'success'
        else:
            self.node.get_logger().error(f'FilterPickClassDetections: No detections ({detection_msg.message})')
            return 'failure'


class SelectNextPickClassDetectionState(State):
    """State: Select the next class CLASS_ID_PICK detection from the list"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['has_more', 'all_done', 'failure'],
            input_keys=['all_detections', 'current_index'],
            output_keys=['centroid', 'depth', 'orientation']
        )
        self.node = node
    
    def execute(self, userdata):
        """Select next detection from the list (doesn't increment index)"""
        
        if not userdata.all_detections or len(userdata.all_detections) == 0:
            self.node.get_logger().error('SelectNextPickClassDetection: No detections available')
            return 'failure'
        
        if userdata.current_index >= len(userdata.all_detections):
            self.node.get_logger().info('SelectNextPickClassDetection: All detections processed')
            return 'all_done'
        
        detection = userdata.all_detections[userdata.current_index]
        
        userdata.centroid = detection.centroid
        userdata.depth = detection.depth
        userdata.orientation = detection.orientation
        
        self.node.get_logger().info(
            f'SelectNextPickClassDetection: Selected detection {userdata.current_index}/{len(userdata.all_detections)-1} '
            f'at ({detection.centroid.x:.1f}, {detection.centroid.y:.1f}), depth={detection.depth:.3f}m'
        )
        
        return 'has_more'


class GetPlacementState(State):
    """State: Get placement location using placement service"""
    
    def __init__(self, node, stack=False):
        State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['all_detections', 'current_index'],
            output_keys=['placement_centroid', 'placement_depth']
        )
        self.node = node
        self.stack = stack
        self.client = self.node.create_client(GetPlacement, '/get_placement')
    
    def execute(self, userdata):
        """Call placement service to get placement location"""
        
        self.node.get_logger().info(f'GetPlacementState: Calling placement service (stack={self.stack})')
        
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Placement service not available')
            return 'failure'
        
        # Get current detection
        if not userdata.all_detections or userdata.current_index >= len(userdata.all_detections):
            self.node.get_logger().error('GetPlacementState: Invalid detection index')
            return 'failure'
        
        detection = userdata.all_detections[userdata.current_index]
        
        # Create request
        request = GetPlacement.Request()
        request.detections = userdata.all_detections
        request.detection_index = userdata.current_index
        request.stack = self.stack
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        
        if not future.done():
            self.node.get_logger().error('Placement service call timed out')
            return 'failure'
        
        response = future.result()
        
        if not response.success:
            self.node.get_logger().error(f'Placement failed: {response.message}')
            return 'failure'
        
        # Store placement location
        userdata.placement_centroid = response.centroid
        userdata.placement_depth = response.depth
        
        self.node.get_logger().info(
            f'GetPlacementState: Found placement at ({response.centroid.x:.1f}, {response.centroid.y:.1f}), '
            f'depth={response.depth:.3f}m'
        )
        
        return 'success'


class ProjectPlacementState(State):
    """State: Project placement 2D pixel coordinates to 3D world coordinates"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['placement_centroid', 'placement_depth'],
            output_keys=['placement_world_coords']
        )
        self.node = node
        self.client = self.node.create_client(ProjectTo3D, '/project_to_3d')
    
    def execute(self, userdata):
        """Project placement location to 3D"""
        
        self.node.get_logger().info('ProjectPlacementState: Projecting placement to 3D')
        
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Project to 3D service not available')
            return 'failure'
        
        # Create request
        request = ProjectTo3D.Request()
        request.centroid = userdata.placement_centroid
        request.depth = userdata.placement_depth
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        if not future.done():
            self.node.get_logger().error('Project to 3D service call timed out')
            return 'failure'
        
        response = future.result()
        
        if not response.success:
            self.node.get_logger().error(f'Projection failed: {response.message}')
            return 'failure'
        
        # Store 3D coordinates
        userdata.placement_world_coords = response.world_coords
        
        self.node.get_logger().info(
            f'ProjectPlacementState: Placement at ({response.world_coords.x:.3f}, '
            f'{response.world_coords.y:.3f}, {response.world_coords.z:.3f})'
        )
        
        return 'success'


class PlaceOnTableState(State):
    """State: Place object on table using placement location"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['placement_world_coords']
        )
        self.node = node
        self.move_to_pose_client = self.node.create_client(MoveToPose, '/move_to_pose')
        self.gripper_action_client = ActionClient(self.node, GripperCommand, 'gripper_controller/gripper_cmd')
        config = GripperConfig()
        self.open_position_rad = 0.0
        self.close_position_rad = config.max_position_rad * 0.98
    
    def execute(self, userdata):
        """Execute place sequence: pre-grasp -> drop position -> wait -> open -> retract -> home -> close"""
        
        self.node.get_logger().info('PlaceOnTableState: Moving to placement location')
        
        # Wait for services
        if not self.move_to_pose_client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error('Move to pose service not available')
            return 'failure'
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Gripper action server not available')
            return 'failure'
        
        # Create orientation (gripper pointing down)
        rotation = R.from_rotvec([0.0, -math.pi, 0.0])
        quat = rotation.as_quat()
        drop_orientation = Quaternion()
        drop_orientation.x = quat[0]
        drop_orientation.y = quat[1]
        drop_orientation.z = quat[2]
        drop_orientation.w = quat[3]
        
        # Step 1: Move to pre-grasp position
        self.node.get_logger().info('PlaceOnTableState: Moving to pre-grasp above placement location')
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = 'ur10_base'
        pre_grasp_pose.pose.position.x = userdata.placement_world_coords.x
        pre_grasp_pose.pose.position.y = userdata.placement_world_coords.y
        pre_grasp_pose.pose.position.z = userdata.placement_world_coords.z + PRE_GRASP_HEIGHT
        pre_grasp_pose.pose.orientation = drop_orientation
        
        move_request = MoveToPose.Request()
        move_request.target_pose = pre_grasp_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to move to pre-grasp')
            return 'failure'
        
        # Step 2: Move to drop position (2.5cm above placement location)
        self.node.get_logger().info('PlaceOnTableState: Moving to drop position (2.5cm above)')
        drop_pose = PoseStamped()
        drop_pose.header.frame_id = 'ur10_base'
        drop_pose.pose.position.x = userdata.placement_world_coords.x
        drop_pose.pose.position.y = userdata.placement_world_coords.y
        drop_pose.pose.position.z = userdata.placement_world_coords.z + DROP_OFFSET_Z
        drop_pose.pose.orientation = drop_orientation
        
        move_request = MoveToPose.Request()
        move_request.target_pose = drop_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to move to drop position')
            return 'failure'
        
        # Step 3: Wait 1 second at drop position
        self.node.get_logger().info('PlaceOnTableState: Waiting 1 second at drop position')
        time.sleep(1.0)
        
        # Step 4: Open gripper to drop
        if not self._send_gripper_goal(open_gripper=True):
            return 'failure'
        
        # Step 5: Retract to pre-grasp
        self.node.get_logger().info('PlaceOnTableState: Retracting to pre-grasp')
        move_request = MoveToPose.Request()
        move_request.target_pose = pre_grasp_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to retract to pre-grasp')
            return 'failure'
        
        # Step 6: Move to home
        self.node.get_logger().info('PlaceOnTableState: Moving to home')
        HOME_X = 0.1215
        HOME_Y = -0.745
        HOME_Z = 0.32325
        HOME_RX = 0.0
        HOME_RY = -math.pi
        HOME_RZ = 0.0
        
        home_rotation = R.from_rotvec([HOME_RX, HOME_RY, HOME_RZ])
        home_quat = home_rotation.as_quat()
        home_orientation = Quaternion()
        home_orientation.x = home_quat[0]
        home_orientation.y = home_quat[1]
        home_orientation.z = home_quat[2]
        home_orientation.w = home_quat[3]
        
        home_pose = PoseStamped()
        home_pose.header.frame_id = 'ur10_base'
        home_pose.pose.position.x = HOME_X
        home_pose.pose.position.y = HOME_Y
        home_pose.pose.position.z = HOME_Z
        home_pose.pose.orientation = home_orientation
        
        move_request = MoveToPose.Request()
        move_request.target_pose = home_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to move to home')
            return 'failure'
        
        # Step 7: Close gripper at home
        if not self._send_gripper_goal(open_gripper=False, warn_only=True):
            self.node.get_logger().warn('Failed to close gripper, but placement completed')
        
        self.node.get_logger().info('PlaceOnTableState: Successfully placed object and moved to home')
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


class IncrementIndexState(State):
    """State: Increment current_index for next detection"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success'],
            input_keys=['current_index'],
            output_keys=['current_index']
        )
        self.node = node
    
    def execute(self, userdata):
        """Increment index"""
        userdata.current_index += 1
        self.node.get_logger().info(f'IncrementIndexState: Incremented to index {userdata.current_index}')
        return 'success'


class CheckClass2ObjectsState(State):
    """State: Check if there are any class 2 objects remaining"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['has_class2', 'no_class2', 'failure'],
            output_keys=['has_class2']
        )
        self.node = node
        self.segmentation_client = self.node.create_client(GetSegmentation, '/get_segmentation')
        self.node = node
    
    def execute(self, userdata):
        """Check for class 2 objects"""
        
        if not self.segmentation_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('CheckClass2Objects: Segmentation service unavailable')
            userdata.has_class2 = False
            return 'no_class2'

        try:
            request = GetSegmentation.Request()
            future = self.segmentation_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        except Exception as exc:
            self.node.get_logger().error(f'CheckClass2Objects: Segmentation call failed: {exc}')
            userdata.has_class2 = False
            return 'no_class2'

        self.node.get_logger().info('CheckClass2Objects: Checking /detections topic for class 2 objects')
        
        detection_msg = self.node.wait_for_detections(timeout_sec=5.0)
        if detection_msg is None:
            self.node.get_logger().warn('CheckClass2Objects: No detections received (timeout)')
            userdata.has_class2 = False
            return 'no_class2'
        
        if detection_msg.success and len(detection_msg.detections) > 0:
            class2_detections = [d for d in detection_msg.detections if d.class_id == CLASS_ID_PICK]
            
            if len(class2_detections) > 0:
                self.node.get_logger().info(
                    f'CheckClass2Objects: Found {len(class2_detections)} class 2 object(s) - continuing'
                )
                userdata.has_class2 = True
                return 'has_class2'
            else:
                self.node.get_logger().info('CheckClass2Objects: No class 2 objects found - done')
                userdata.has_class2 = False
                return 'no_class2'
        else:
            self.node.get_logger().warn(f'CheckClass2Objects: No detections or segmentation failed ({detection_msg.message})')
            userdata.has_class2 = False
            return 'no_class2'


class PickAndPlaceNHCSequencerNode(Node):
    """SMACH state machine for picking class 2 objects and placing on table using placement service"""
    
    def __init__(self, stack=False):
        super().__init__('pick_and_place_nhc_sequencer')
        self.stack = stack
        
        # Create SMACH state machine
        self.latest_detection_msg = None
        self.detections_event = threading.Event()
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.detections_sub = self.create_subscription(
            DetectionArray,
            '/detections',
            self._detections_callback,
            qos
        )
        
        self.sm = smach.StateMachine(outcomes=['success', 'failure'])
        
        # Define userdata
        self.sm.userdata.all_detections = []
        self.sm.userdata.current_index = 0
        self.sm.userdata.centroid = None
        self.sm.userdata.depth = None
        self.sm.userdata.orientation = None
        self.sm.userdata.world_coords = None
        self.sm.userdata.placement_centroid = None
        self.sm.userdata.placement_depth = None
        self.sm.userdata.placement_world_coords = None
        
        # Build state machine
        with self.sm:
            # Move to home position first
            smach.StateMachine.add(
                'HOME',
                HomeState(self),
                transitions={
                    'success': 'FILTER_PICK_CLASS',
                    'failure': 'failure'
                }
            )
            
            # Filter detections for pick class (CLASS_ID_PICK = 2) only
            smach.StateMachine.add(
                'FILTER_PICK_CLASS',
                FilterPickClassDetectionsState(self),
                transitions={
                    'success': 'SELECT_NEXT_PICK_CLASS',
                    'no_objects': 'CHECK_CLASS2',
                    'failure': 'failure'
                }
            )
            
            # Select next pick class object to pick
            smach.StateMachine.add(
                'SELECT_NEXT_PICK_CLASS',
                SelectNextPickClassDetectionState(self),
                transitions={
                    'has_more': 'PROJECT',
                    'all_done': 'CHECK_CLASS2',
                    'failure': 'failure'
                }
            )
            
            # Project 2D to 3D for pick location
            smach.StateMachine.add(
                'PROJECT',
                ProjectState(self),
                transitions={
                    'success': 'PICK',
                    'failure': 'failure'
                }
            )
            
            # Pick the object
            smach.StateMachine.add(
                'PICK',
                PickState(self, release_after_pick=False),
                transitions={
                    'success': 'HOME_AFTER_PICK',
                    'failure': 'failure'
                }
            )
            
            # Move to home after picking (for better camera view)
            smach.StateMachine.add(
                'HOME_AFTER_PICK',
                HomeState(self),
                transitions={
                    'success': 'GET_PLACEMENT',
                    'failure': 'failure'
                }
            )
            
            # Get placement location using placement service
            smach.StateMachine.add(
                'GET_PLACEMENT',
                GetPlacementState(self, stack=self.stack),
                transitions={
                    'success': 'PROJECT_PLACEMENT',
                    'failure': 'failure'
                }
            )
            
            # Project placement 2D to 3D
            smach.StateMachine.add(
                'PROJECT_PLACEMENT',
                ProjectPlacementState(self),
                transitions={
                    'success': 'PLACE_ON_TABLE',
                    'failure': 'failure'
                }
            )
            
            # Place the object on table
            smach.StateMachine.add(
                'PLACE_ON_TABLE',
                PlaceOnTableState(self),
                transitions={
                    'success': 'INCREMENT_INDEX',
                    'failure': 'failure'
                }
            )
            
            # Increment index and continue
            smach.StateMachine.add(
                'INCREMENT_INDEX',
                IncrementIndexState(self),
                transitions={
                    'success': 'FILTER_PICK_CLASS'  # Re-detect after placing
                }
            )
            
            # Check for class 2 objects to determine if we should continue
            smach.StateMachine.add(
                'CHECK_CLASS2',
                CheckClass2ObjectsState(self),
                transitions={
                    'has_class2': 'FILTER_PICK_CLASS',  # Continue if class 2 exists
                    'no_class2': 'success',  # Done if no class 2
                    'failure': 'success'  # Treat failure as completion
                }
            )
        
        self.get_logger().info('Pick-and-place-NHC sequencer initialized')
        
        # Execute state machine
        self.execute()
    
    def execute(self):
        """Execute the state machine"""
        
        self.get_logger().info('Starting SMACH state machine for pick-and-place-NHC')
        
        outcome = self.sm.execute()
        
        self.get_logger().info(f'State machine finished with outcome: {outcome}')

    def _detections_callback(self, msg: DetectionArray):
        self.latest_detection_msg = msg
        self.detections_event.set()

    def wait_for_detections(self, timeout_sec: float = 5.0):
        if not self.detections_event.wait(timeout_sec):
            return None
        self.detections_event.clear()
        return self.latest_detection_msg


def main(args=None):
    import argparse
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Pick and place NHC sequencer')
    parser.add_argument(
        '--stack',
        action='store_true',
        help='Allow stacking on other objects (default: False, only place on table)'
    )
    
    # Parse known args (ROS 2 will handle the rest)
    known_args, ros_args = parser.parse_known_args(args)
    
    rclpy.init(args=ros_args)
    
    sequencer = PickAndPlaceNHCSequencerNode(stack=known_args.stack)
    
    try:
        rclpy.spin(sequencer)
    except KeyboardInterrupt:
        pass
    finally:
        sequencer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

