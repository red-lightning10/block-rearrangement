import rclpy
import sys
from rclpy.node import Node
from interfaces.srv import GetSegmentation, GetPlacement
from geometry_msgs.msg import Point


class PlacementClient(Node):
    """Client to test placement service"""
    
    def __init__(self):
        super().__init__('placement_client')
        
        # Service clients
        self.segmentation_client = self.create_client(
            GetSegmentation,
            '/get_segmentation'
        )
        
        self.placement_client = self.create_client(
            GetPlacement,
            '/get_placement'
        )
        
        self.get_logger().info('Placement Client started')
    
    def run(self, stack=False):
        """Run the full pipeline: segmentation -> placement
        
        Args:
            stack: If True, allow stacking on other objects; if False, only place on table
        """
        
        # Step 1: Call segmentation service
        self.get_logger().info('Calling segmentation service...')
        if not self.segmentation_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Segmentation service not available')
            return
        
        seg_request = GetSegmentation.Request()
        seg_future = self.segmentation_client.call_async(seg_request)
        rclpy.spin_until_future_complete(self, seg_future, timeout_sec=10.0)
        
        if not seg_future.done():
            self.get_logger().error('Segmentation service call timed out')
            return
        
        seg_response = seg_future.result()
        if not seg_response.success:
            self.get_logger().error(f'Segmentation failed: {seg_response.message}')
            return
        
        if len(seg_response.detections) == 0:
            self.get_logger().warn('No detections found')
            return
        
        self.get_logger().info(f'Found {len(seg_response.detections)} detection(s)')
        
        # Step 2: Call placement service for first detection (index 0)
        detection_index = 0
        self.get_logger().info(f'Calling placement service for detection index {detection_index}...')
        
        if not self.placement_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Placement service not available')
            return
        
        placement_request = GetPlacement.Request()
        placement_request.detections = seg_response.detections
        placement_request.detection_index = detection_index
        placement_request.stack = stack
        
        placement_future = self.placement_client.call_async(placement_request)
        rclpy.spin_until_future_complete(self, placement_future, timeout_sec=30.0)
        
        if not placement_future.done():
            self.get_logger().error('Placement service call timed out')
            return
        
        placement_response = placement_future.result()
        
        if placement_response.success:
            centroid = placement_response.centroid
            depth = placement_response.depth
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('PLACEMENT RESULT:')
            self.get_logger().info(f'  Centroid (pixels): x={centroid.x:.1f}, y={centroid.y:.1f}')
            self.get_logger().info(f'  Depth: {depth:.3f} m')
            self.get_logger().info('=' * 50)
            
            print(f'\nPlacement Location:')
            print(f'  Pixel coordinates: ({centroid.x:.1f}, {centroid.y:.1f})')
            print(f'  Depth: {depth:.3f} m')
        else:
            self.get_logger().error(f'Placement failed: {placement_response.message}')


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments for stack parameter
    stack = False
    if len(sys.argv) > 1:
        if '--stack' in sys.argv or 'stack=true' in sys.argv or 'stack=True' in sys.argv:
            stack = True
        elif 'stack=false' in sys.argv or 'stack=False' in sys.argv:
            stack = False
    
    client = PlacementClient()
    
    try:
        client.run(stack=stack)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

