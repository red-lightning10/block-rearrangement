import rclpy
from rclpy.node import Node
import smach
from smach import State
from sequencer.states import HomeState, ProjectState, PickState
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from interfaces.srv import ControlGripper, MoveToPose, GetSegmentation, ProjectTo3D
import math
import time
from scipy.spatial.transform import Rotation as R

# Class IDs
CLASS_ID_PICK = 2  # Objects to pick
CLASS_ID_PLACE = 4  # Drop location targets

# Drop offset: 2.5 cm above the class 4 object
DROP_OFFSET_Z = 0.025  # 2.5 cm in meters
PRE_GRASP_HEIGHT = 0.1  # 10 cm above for pre-grasp position


class FilterPickClassDetectionsState(State):
    """State: Filter detections to only class_id == CLASS_ID_PICK and store them"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'no_objects', 'failure'],
            input_keys=['all_detections'],  # Need to read to clear previous detections
            output_keys=['all_detections', 'current_index']
        )
        self.node = node
        self.client = self.node.create_client(
            GetSegmentation,
            '/get_segmentation'
        )
    
    def execute(self, userdata):
        """Call segmentation service and filter for class_id == CLASS_ID_PICK"""
        
        self.node.get_logger().info(f'FilterPickClassDetections: Calling segmentation service (filtering for class_id={CLASS_ID_PICK})')
        
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Segmentation service not available')
            return 'failure'
        
        # Create request
        request = GetSegmentation.Request()
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        if not future.done():
            self.node.get_logger().error('Segmentation service call timed out')
            return 'failure'
        
        response = future.result()
        
        if response.success and len(response.detections) > 0:
            # Clear previous detections to ensure fresh data
            old_count = len(userdata.all_detections) if userdata.all_detections else 0

            # Clear other detection-related fields
            userdata.centroid = None
            userdata.depth = None
            userdata.orientation = None
            userdata.world_coords = None
            
            if old_count > 0:
                self.node.get_logger().info(f'FilterPickClassDetections: Clearing {old_count} previous detection(s)')
            
            # Filter for class_id == CLASS_ID_PICK only
            pick_class_detections = [d for d in response.detections if d.class_id == CLASS_ID_PICK]

            if len(pick_class_detections) == 0:
                self.node.get_logger().info(f'FilterPickClassDetections: No class {CLASS_ID_PICK} objects found')
                return 'no_objects'
            
            # Store filtered detections (replacing any previous detections)
            userdata.all_detections = pick_class_detections
            userdata.current_index = 0
            
            self.node.get_logger().info(
                f'FilterPickClassDetections: Found {len(pick_class_detections)} class {CLASS_ID_PICK} object(s) to pick (replaced previous detections)'
            )
            return 'success'
        else:
            self.node.get_logger().error(f'Segmentation failed: {response.message}')
            return 'failure'


class SelectNextPickClassDetectionState(State):
    """State: Select the next class CLASS_ID_PICK detection from the list"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['has_more', 'all_done', 'failure'],
            input_keys=['all_detections', 'current_index'],
            output_keys=['centroid', 'depth', 'orientation', 'current_index']
        )
        self.node = node
    
    def execute(self, userdata):
        """Select next detection or signal completion"""
        
        # Check if we have detections
        if not userdata.all_detections or len(userdata.all_detections) == 0:
            self.node.get_logger().error('No detections available')
            return 'failure'
        
        # Check if we've processed all detections
        if userdata.current_index >= len(userdata.all_detections):
            self.node.get_logger().info(f'All class {CLASS_ID_PICK} objects have been picked!')
            return 'all_done'
        
        # Get current detection
        current_detection = userdata.all_detections[userdata.current_index]
        
        # Store in userdata for next states
        userdata.centroid = current_detection.centroid
        userdata.depth = current_detection.depth
        userdata.orientation = current_detection.orientation
        
        self.node.get_logger().info(
            f'SelectNextPickClassDetection: Processing object {userdata.current_index + 1}/{len(userdata.all_detections)} '
            f'(class_id={current_detection.class_id}, confidence={current_detection.confidence:.2f})'
        )
        
        # Increment index for next iteration
        userdata.current_index += 1
        
        return 'has_more'


class FindClass4DropLocationState(State):
    """State: Find a class 4 detection to use as drop location"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'no_drop_location', 'failure'],
            output_keys=['drop_world_coords', 'drop_orientation']
        )
        self.node = node
        self.segmentation_client = self.node.create_client(
            GetSegmentation,
            '/get_segmentation'
        )
        self.project_client = self.node.create_client(
            ProjectTo3D,
            '/project_to_3d'
        )
    
    def execute(self, userdata):
        """Find class 4 detection and project to 3D world coordinates"""
        
        self.node.get_logger().info(f'FindClass4DropLocation: Looking for class {CLASS_ID_PLACE} objects (fresh detection)')
        
        # Clear previous drop location to ensure fresh data
        userdata.drop_world_coords = None
        userdata.drop_orientation = None
        
        # Wait for services
        if not self.segmentation_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Segmentation service not available')
            return 'failure'
        
        if not self.project_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Project to 3D service not available')
            return 'failure'
        
        # Call segmentation (fresh detection)
        request = GetSegmentation.Request()
        future = self.segmentation_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        if not future.done():
            self.node.get_logger().error('Segmentation service call timed out')
            return 'failure'
        
        response = future.result()
        
        if not response.success or len(response.detections) == 0:
            self.node.get_logger().error('No detections found')
            return 'no_drop_location'
        
        # Filter for class_id == CLASS_ID_PLACE
        class4_detections = [d for d in response.detections if d.class_id == CLASS_ID_PLACE]
        
        if len(class4_detections) == 0:
            self.node.get_logger().warn(f'FindClass4DropLocation: No class {CLASS_ID_PLACE} objects found for drop location')
            return 'no_drop_location'
        
        # Use the highest confidence class 4 detection
        drop_detection = max(class4_detections, key=lambda d: d.confidence)
        
        self.node.get_logger().info(
            f'FindClass4DropLocation: Found class {CLASS_ID_PLACE} object at ({drop_detection.centroid.x:.1f}, {drop_detection.centroid.y:.1f}) '
            f'(confidence={drop_detection.confidence:.2f})'
        )
        
        # Project to 3D world coordinates
        project_request = ProjectTo3D.Request()
        project_request.centroid = drop_detection.centroid
        project_request.depth = drop_detection.depth
        
        project_future = self.project_client.call_async(project_request)
        rclpy.spin_until_future_complete(self.node, project_future, timeout_sec=10.0)
        
        if not project_future.done():
            self.node.get_logger().error('Project to 3D service call timed out')
            return 'failure'
        
        project_response = project_future.result()
        
        if not project_response.success:
            self.node.get_logger().error(f'Projection failed: {project_response.message}')
            return 'failure'
        
        # Store drop location (will be adjusted with offset in place state)
        userdata.drop_world_coords = project_response.world_coords
        userdata.drop_orientation = drop_detection.orientation
        
        self.node.get_logger().info(
            f'FindClass4DropLocation: Drop location at ({project_response.world_coords.x:.3f}, '
            f'{project_response.world_coords.y:.3f}, {project_response.world_coords.z:.3f})'
        )
        
        return 'success'


class PlaceOnClass4State(State):
    """State: Place object on class 4 location with 2.5cm offset above"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['drop_world_coords', 'drop_orientation']
        )
        self.node = node
        self.move_to_pose_client = self.node.create_client(MoveToPose, '/move_to_pose')
        self.gripper_client = self.node.create_client(ControlGripper, '/control_gripper')
    
    def execute(self, userdata):
        """Execute place sequence: pre-grasp above class 4 -> 2.5cm above -> wait 1s -> open -> retract to pre-grasp -> go home -> close gripper"""
        
        self.node.get_logger().info('PlaceOnClass4State: Moving to class 4 drop location')
        
        # Wait for services
        if not self.move_to_pose_client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error('Move to pose service not available')
            return 'failure'
        
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Control gripper service not available')
            return 'failure'
        
        # Create orientation (gripper pointing down)
        rotation = R.from_rotvec([0.0, -math.pi, 0.0])
        quat = rotation.as_quat()
        drop_orientation = Quaternion()
        drop_orientation.x = quat[0]
        drop_orientation.y = quat[1]
        drop_orientation.z = quat[2]
        drop_orientation.w = quat[3]
        
        # Step 1: Move to pre-grasp position (PRE_GRASP_HEIGHT above class 4)
        self.node.get_logger().info('PlaceOnClass4State: Moving to pre-grasp above class 4')
        pre_grasp_pose = PoseStamped()
        pre_grasp_pose.header.frame_id = 'ur10_base'
        pre_grasp_pose.pose.position.x = userdata.drop_world_coords.x
        pre_grasp_pose.pose.position.y = userdata.drop_world_coords.y
        pre_grasp_pose.pose.position.z = userdata.drop_world_coords.z + PRE_GRASP_HEIGHT
        pre_grasp_pose.pose.orientation = drop_orientation
        
        move_request = MoveToPose.Request()
        move_request.target_pose = pre_grasp_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to move to pre-grasp above class 4')
            return 'failure'
        
        # Step 2: Move to drop position (2.5cm above class 4, not all the way down)
        self.node.get_logger().info('PlaceOnClass4State: Moving to drop position (2.5cm above class 4)')
        drop_pose = PoseStamped()
        drop_pose.header.frame_id = 'ur10_base'
        drop_pose.pose.position.x = userdata.drop_world_coords.x
        drop_pose.pose.position.y = userdata.drop_world_coords.y
        drop_pose.pose.position.z = userdata.drop_world_coords.z + DROP_OFFSET_Z  # 2.5 cm above
        drop_pose.pose.orientation = drop_orientation
        
        move_request = MoveToPose.Request()
        move_request.target_pose = drop_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to move to drop position')
            return 'failure'
        
        # Step 3: Wait 1 second at drop position
        self.node.get_logger().info('PlaceOnClass4State: Waiting 1 second at drop position')
        time.sleep(1.0)
        
        # Step 4: Open gripper to drop
        self.node.get_logger().info('PlaceOnClass4State: Opening gripper to release object')
        gripper_request = ControlGripper.Request()
        gripper_request.command = 'open'
        gripper_future = self.gripper_client.call_async(gripper_request)
        rclpy.spin_until_future_complete(self.node, gripper_future, timeout_sec=10.0)
        if not gripper_future.done() or not gripper_future.result().success:
            self.node.get_logger().error('Failed to open gripper')
            return 'failure'
        
        # Step 5: Retract to pre-grasp (to avoid knocking things over)
        self.node.get_logger().info('PlaceOnClass4State: Retracting to pre-grasp')
        move_request = MoveToPose.Request()
        move_request.target_pose = pre_grasp_pose
        move_future = self.move_to_pose_client.call_async(move_request)
        rclpy.spin_until_future_complete(self.node, move_future, timeout_sec=60.0)
        if not move_future.done() or not move_future.result().success:
            self.node.get_logger().error('Failed to retract to pre-grasp')
            return 'failure'
        
        # Step 6: Move to home (for re-detection)
        self.node.get_logger().info('PlaceOnClass4State: Moving to home for re-detection')
        # Home position constants (matching HomeState)
        HOME_X = 0.1215
        HOME_Y = -0.745
        HOME_Z = 0.32325
        HOME_RX = 0.0
        HOME_RY = -math.pi
        HOME_RZ = 0.0
        
        # Create home pose
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
        self.node.get_logger().info('PlaceOnClass4State: Closing gripper at home')
        gripper_request = ControlGripper.Request()
        gripper_request.command = 'close'
        gripper_future = self.gripper_client.call_async(gripper_request)
        rclpy.spin_until_future_complete(self.node, gripper_future, timeout_sec=10.0)
        if not gripper_future.done() or not gripper_future.result().success:
            self.node.get_logger().warn('Failed to close gripper, but placement completed')
        
        self.node.get_logger().info('PlaceOnClass4State: Successfully placed object on class 4 and moved to home')
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
        self.client = self.node.create_client(
            GetSegmentation,
            '/get_segmentation'
        )
    
    def execute(self, userdata):
        """Check for class 2 objects"""
        
        self.node.get_logger().info('CheckClass2Objects: Checking for class 2 objects')
        
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Segmentation service not available')
            return 'failure'
        
        # Create request
        request = GetSegmentation.Request()
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        if not future.done():
            self.node.get_logger().error('Segmentation service call timed out')
            return 'failure'
        
        response = future.result()
        
        if response.success and len(response.detections) > 0:
            # Filter for class_id == CLASS_ID_PICK
            class2_detections = [d for d in response.detections if d.class_id == CLASS_ID_PICK]
            
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
            self.node.get_logger().warn('CheckClass2Objects: No detections or segmentation failed')
            userdata.has_class2 = False
            return 'no_class2'


class PickAndPlaceHCSequencerNode(Node):
    """SMACH state machine for picking class 3 objects and placing on class 4 locations"""
    
    def __init__(self):
        super().__init__('pick_and_place_hc_sequencer')
        
        # Create SMACH state machine
        self.sm = smach.StateMachine(outcomes=['success', 'failure'])
        
        # Define userdata
        self.sm.userdata.all_detections = []
        self.sm.userdata.current_index = 0
        self.sm.userdata.centroid = None
        self.sm.userdata.depth = None
        self.sm.userdata.orientation = None
        self.sm.userdata.world_coords = None
        self.sm.userdata.drop_world_coords = None
        self.sm.userdata.drop_orientation = None
        self.sm.userdata.has_class2 = False
        
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
            
            # Filter detections for pick class (CLASS_ID_PICK) only
            smach.StateMachine.add(
                'FILTER_PICK_CLASS',
                FilterPickClassDetectionsState(self),
                transitions={
                    'success': 'SELECT_NEXT_PICK_CLASS',
                    'no_objects': 'CHECK_CLASS2',  # If no pick class objects, check for class 2
                    'failure': 'failure'
                }
            )
            
            # Select next pick class object to pick
            smach.StateMachine.add(
                'SELECT_NEXT_PICK_CLASS',
                SelectNextPickClassDetectionState(self),
                transitions={
                    'has_more': 'PROJECT',
                    'all_done': 'CHECK_CLASS2',  # Done with pick class, check for class 2
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
            
            # Pick the pick class object (don't release - we'll place it on class 4)
            smach.StateMachine.add(
                'PICK',
                PickState(self, release_after_pick=False),
                transitions={
                    'success': 'HOME_AFTER_PICK',
                    'failure': 'failure'
                }
            )
            
            # Move to home after picking (for better camera view to detect class 4)
            smach.StateMachine.add(
                'HOME_AFTER_PICK',
                HomeState(self),
                transitions={
                    'success': 'FIND_DROP_LOCATION',
                    'failure': 'failure'
                }
            )
            
            # Find class 4 drop location (after going home, so camera has good view)
            smach.StateMachine.add(
                'FIND_DROP_LOCATION',
                FindClass4DropLocationState(self),
                transitions={
                    'success': 'PLACE_ON_CLASS4',
                    'no_drop_location': 'failure',  # Can't proceed without drop location
                    'failure': 'failure'
                }
            )
            
            # Place the object on class 4 location (2.5cm above) and go to home
            smach.StateMachine.add(
                'PLACE_ON_CLASS4',
                PlaceOnClass4State(self),
                transitions={
                    'success': 'FILTER_PICK_CLASS',  # Re-detect after placing and going home
                    'failure': 'failure'
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
        
        self.get_logger().info('Pick-and-place-HC sequencer initialized')
        
        # Execute state machine
        self.execute()
    
    def execute(self):
        """Execute the state machine"""
        
        self.get_logger().info('Starting SMACH state machine for pick-and-place-HC')
        
        outcome = self.sm.execute()
        
        self.get_logger().info(f'State machine finished with outcome: {outcome}')


def main(args=None):
    rclpy.init(args=args)
    
    sequencer = PickAndPlaceHCSequencerNode()
    
    try:
        rclpy.spin(sequencer)
    except KeyboardInterrupt:
        pass
    finally:
        sequencer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

