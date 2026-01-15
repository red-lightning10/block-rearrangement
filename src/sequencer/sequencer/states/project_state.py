from smach import State
import rclpy
from interfaces.srv import ProjectTo3D

class ProjectState(State):
    """State: Project 2D pixel coordinates to 3D world coordinates"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'failure'],
            input_keys=['centroid', 'depth'],
            output_keys=['world_coords']
        )
        self.node = node
        self.client = self.node.create_client(ProjectTo3D, '/project_to_3d')
    
    def execute(self, userdata):
        """Execute projection to 3D world coordinates"""
        
        self.node.get_logger().info('ProjectState: Calling world_coords service')

        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('World coords service not available')
            return 'failure'

        request = ProjectTo3D.Request()
        request.centroid = userdata.centroid
        request.depth = userdata.depth
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        
        if not future.done():
            self.node.get_logger().error('World coords service call timed out')
            return 'failure'
        
        response = future.result()
        
        if response.success:
            userdata.world_coords = response.world_coords
            
            self.node.get_logger().info(
                f'ProjectState: Success! World coords=({response.world_coords.x:.3f}, '
                f'{response.world_coords.y:.3f}, {response.world_coords.z:.3f})'
            )
            return 'success'
        else:
            self.node.get_logger().error(f'Projection failed: {response.message}')
            return 'failure'

