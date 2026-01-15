import rclpy
from rclpy.node import Node
import smach
from sequencer.states import HomeState, DetectState, ProjectState, PickState

class PickSequencerNode(Node):
    """SMACH state machine for pick sequence"""
    
    def __init__(self):
        super().__init__('pick_sequencer')
        
        # Create SMACH state machine
        self.sm = smach.StateMachine(outcomes=['success', 'failure'])
        
        # Define userdata
        self.sm.userdata.centroid = None
        self.sm.userdata.depth = None
        self.sm.userdata.orientation = None
        self.sm.userdata.world_coords = None
        
        # Build state machine
        with self.sm:
            # Add states with node reference
            smach.StateMachine.add(
                'HOME',
                HomeState(self),
                transitions={
                    'success': 'DETECT',
                    'failure': 'failure'
                }
            )
            
            smach.StateMachine.add(
                'DETECT',
                DetectState(self),
                transitions={
                    'success': 'PROJECT',
                    'failure': 'failure'
                }
            )
            
            smach.StateMachine.add(
                'PROJECT',
                ProjectState(self),
                transitions={
                    'success': 'PICK',
                    'failure': 'failure'
                }
            )
            
            smach.StateMachine.add(
                'PICK',
                PickState(self),
                transitions={
                    'success': 'success',
                    'failure': 'failure'
                }
            )
        
        self.get_logger().info('Pick sequencer initialized')
        
        # Execute state machine
        self.execute()
    
    def execute(self):
        """Execute the state machine"""
        
        self.get_logger().info('Starting SMACH state machine')
        
        outcome = self.sm.execute()
        
        self.get_logger().info(f'State machine finished with outcome: {outcome}')


def main(args=None):
    rclpy.init(args=args)
    
    sequencer = PickSequencerNode()
    
    try:
        rclpy.spin(sequencer)
    except KeyboardInterrupt:
        pass
    finally:
        sequencer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

