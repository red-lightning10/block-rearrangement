from smach import State
import rclpy
from interfaces.srv import GetSegmentation

class DetectState(State):
    """State: Detect and segment object to get 2D coordinates"""
    
    def __init__(self, node):
        State.__init__(
            self,
            outcomes=['success', 'failure'],
            output_keys=['centroid', 'depth', 'orientation']
        )
        self.node = node
        self.client = self.node.create_client(GetSegmentation, '/get_segmentation')
    
    def execute(self, userdata):
        """Execute detection and segmentation"""
        
        self.node.get_logger().info('DetectState: Calling segmentation service')
        
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Segmentation service not available')
            return 'failure'
        
        request = GetSegmentation.Request()
        
        # Detectron2 inference can take 5-15+ seconds depending on image complexity
        # Use a longer timeout to avoid premature failures
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        
        if not future.done():
            self.node.get_logger().error('Segmentation service call timed out')
            return 'failure'
        
        response = future.result()
        
        if response.success and len(response.detections) > 0:
            
            # NOTE: For now we pick the highest confidence detection
            best_detection = max(response.detections, key=lambda d: d.confidence)

            userdata.centroid = best_detection.centroid
            userdata.depth = best_detection.depth
            userdata.orientation = best_detection.orientation
            
            self.node.get_logger().info(
                f'DetectState: Found {len(response.detections)} object(s), '
                f'picking best (confidence={best_detection.confidence:.2f})'
            )
            self.node.get_logger().info(
                f'DetectState: Centroid=({best_detection.centroid.x:.1f}, {best_detection.centroid.y:.1f}), '
                f'Depth={best_detection.depth:.3f}m, Orientation={best_detection.orientation:.2f}°'
            )
            return 'success'
        else:
            self.node.get_logger().error(f'Segmentation failed: {response.message}')
            return 'failure'

