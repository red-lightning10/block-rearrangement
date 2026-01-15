import rclpy
from rclpy.node import Node
from interfaces.srv import GetSegmentation


class SegmentationTrigger(Node):
    def __init__(self):
        super().__init__('segmentation_trigger')
        self.client = self.create_client(GetSegmentation, '/get_segmentation')
        
    def trigger(self):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Segmentation service not available')
            return False
            
        request = GetSegmentation.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if not future.done():
            self.get_logger().error('Segmentation service call timed out')
            return False
            
        response = future.result()
        
        if response.success:
            self.get_logger().info(f'Segmentation successful: {response.message}')
            self.get_logger().info(f'Found {len(response.detections)} detection(s)')
            for i, det in enumerate(response.detections):
                self.get_logger().info(
                    f'  Detection {i}: class_id={det.class_id}, '
                    f'centroid=({det.centroid.x:.1f}, {det.centroid.y:.1f}), '
                    f'depth={det.depth:.3f}m'
                )
            return True
        else:
            self.get_logger().error(f'Segmentation failed: {response.message}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    trigger = SegmentationTrigger()
    success = trigger.trigger()
    
    trigger.destroy_node()
    rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())



