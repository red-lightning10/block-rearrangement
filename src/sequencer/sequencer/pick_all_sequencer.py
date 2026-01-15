import rclpy
from rclpy.node import Node
import smach
from smach import State
from sequencer.states import HomeState, ProjectState, PickState, PlaceState

DROP_LOCATION = {'x': -0.560, 'y': -0.375, 'z': 0.225}

class SelectNextDetectionState(State):
    """State: Select the next detection from the list"""
    
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
            self.node.get_logger().info('All objects have been picked!')
            return 'all_done'
        
        # Get current detection
        current_detection = userdata.all_detections[userdata.current_index]
        
        # Store in userdata for next states
        userdata.centroid = current_detection.centroid
        userdata.depth = current_detection.depth
        userdata.orientation = current_detection.orientation
        
        self.node.get_logger().info(
            f'SelectNextDetection: Processing object {userdata.current_index + 1}/{len(userdata.all_detections)} '
            f'(confidence={current_detection.confidence:.2f})'
        )
        
        # Increment index for next iteration
        userdata.current_index += 1
        
        return 'has_more'


class StoreAllDetectionsState(State):
    """State: Store all detections from segmentation response"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'failure'],
            output_keys=['all_detections', 'current_index']
        )
        self.node = node
        self.client = self.node.create_client(
            __import__('interfaces.srv', fromlist=['GetSegmentation']).GetSegmentation,
            '/get_segmentation'
        )
    
    def execute(self, userdata):
        """Call segmentation service and store all detections"""
        
        self.node.get_logger().info('StoreAllDetections: Calling segmentation service')
        
        # Wait for service
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Segmentation service not available')
            return 'failure'
        
        # Create request (empty request)
        from interfaces.srv import GetSegmentation
        request = GetSegmentation.Request()
        
        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        if not future.done():
            self.node.get_logger().error('Segmentation service call timed out')
            return 'failure'
        
        response = future.result()
        
        if response.success and len(response.detections) > 0:
            # Store ALL detections
            userdata.all_detections = response.detections
            userdata.current_index = 0  # Start from first detection
            
            self.node.get_logger().info(
                f'StoreAllDetections: Found {len(response.detections)} object(s) to pick'
            )
            return 'success'
        else:
            self.node.get_logger().error(f'Segmentation failed: {response.message}')
            return 'failure'


class PickAllSequencerNode(Node):
    """SMACH state machine for picking all detected objects"""
    
    def __init__(self):
        super().__init__('pick_all_sequencer')
        
        # Create SMACH state machine
        self.sm = smach.StateMachine(outcomes=['success', 'failure'])
        
        # Define userdata
        self.sm.userdata.all_detections = []
        self.sm.userdata.current_index = 0
        self.sm.userdata.centroid = None
        self.sm.userdata.depth = None
        self.sm.userdata.orientation = None
        self.sm.userdata.world_coords = None
        
        # Build state machine
        with self.sm:
            # Move to home position first
            smach.StateMachine.add(
                'HOME',
                HomeState(self),
                transitions={
                    'success': 'DETECT_ALL',
                    'failure': 'failure'
                }
            )
            
            # Detect all objects and store them
            smach.StateMachine.add(
                'DETECT_ALL',
                StoreAllDetectionsState(self),
                transitions={
                    'success': 'SELECT_NEXT',
                    'failure': 'failure'
                }
            )
            
            # Select next object to pick
            smach.StateMachine.add(
                'SELECT_NEXT',
                SelectNextDetectionState(self),
                transitions={
                    'has_more': 'PROJECT',
                    'all_done': 'success',
                    'failure': 'failure'
                }
            )
            
            # Project 2D to 3D
            smach.StateMachine.add(
                'PROJECT',
                ProjectState(self),
                transitions={
                    'success': 'PICK',
                    'failure': 'failure'
                }
            )
            
            # Pick the object (don't release - we'll place it at drop location)
            smach.StateMachine.add(
                'PICK',
                PickState(self, release_after_pick=False),
                transitions={
                    'success': 'PLACE',
                    'failure': 'failure'
                }
            )
            
            # Place the object at drop location
            smach.StateMachine.add(
                'PLACE',
                PlaceState(self, drop_location=DROP_LOCATION),
                transitions={
                    'success': 'RETURN_HOME',  # Go back home before next detection
                    'failure': 'failure'
                }
            )
            
            # Return to home before detecting next object
            smach.StateMachine.add(
                'RETURN_HOME',
                HomeState(self),
                transitions={
                    'success': 'SELECT_NEXT',  # Loop back to select next object
                    'failure': 'failure'
                }
            )
        
        self.get_logger().info('Pick-all sequencer initialized')
        
        # Execute state machine
        self.execute()
    
    def execute(self):
        """Execute the state machine"""
        
        self.get_logger().info('Starting SMACH state machine for pick-all')
        
        outcome = self.sm.execute()
        
        self.get_logger().info(f'State machine finished with outcome: {outcome}')


def main(args=None):
    rclpy.init(args=args)
    
    sequencer = PickAllSequencerNode()
    
    try:
        rclpy.spin(sequencer)
    except KeyboardInterrupt:
        pass
    finally:
        sequencer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

