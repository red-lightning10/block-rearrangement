import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from interfaces.srv import GetPlacement, ProjectTo3D
from placing.utils.placement import get_placements
import numpy as np
import ros2_numpy as rnp
import cv2
from collections import deque
import os
from datetime import datetime


class PlacementServer(Node):
    """Service server for finding valid placement locations for grasped cubes"""
    
    def __init__(self):
        super().__init__('placement_server')
        
        self.buffer_size = 10
        
        # Subscribers for current scene
        self.aligned_depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self._aligned_depth_callback,
            self.buffer_size
        )
        
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self._color_callback,
            self.buffer_size
        )
        
        # Service server
        self.srv = self.create_service(
            GetPlacement,
            'get_placement',
            self.placement_callback
        )
        
        client_callback_group = ReentrantCallbackGroup()
        self.world_coords_client = self.create_client(
            ProjectTo3D,
            '/project_to_3d',
            callback_group=client_callback_group
        )
        
        self.depth_buffer = deque(maxlen=self.buffer_size)
        self.latest_color_image = None
        self.window_created = False
        self.get_logger().info('Placement Service Server started')
    
    def _aligned_depth_callback(self, msg):
        """Callback for depth images"""
        self.depth_buffer.append(msg)
    
    def _color_callback(self, msg):
        """Callback for color images"""
        self.latest_color_image = msg
    
    def get_object_mask_from_msg(self, detection):
        """Get object mask from mask image"""
        obj_mask_array = rnp.numpify(detection.mask)
        if obj_mask_array.dtype != np.uint8:
            obj_mask_array = (obj_mask_array * 255).astype(np.uint8)
        _, obj_mask = cv2.threshold(obj_mask_array, 127, 255, cv2.THRESH_BINARY)
        return obj_mask
    
    def project_to_3d(self, centroid, depth):
        """
        Call world_coords service to convert 2D pixel coordinates and depth to 3D world coordinates
        
        Args:
            centroid: geometry_msgs/Point with x, y pixel coordinates (z unused)
            depth: float32 depth value in meters
        
        Returns:
            world_coords: geometry_msgs/Point with 3D world coordinates, or None if failed
        """
        if not self.world_coords_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('World coords service not available')
            return None
        
        request = ProjectTo3D.Request()
        request.centroid = centroid
        request.depth = depth
        
        future = self.world_coords_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if not future.done():
            self.get_logger().error('World coords service call timed out')
            return None
        
        response = future.result()
        
        if response.success:
            self.get_logger().info(
                f'Projected to 3D: ({response.world_coords.x:.3f}, '
                f'{response.world_coords.y:.3f}, {response.world_coords.z:.3f})'
            )
            return response.world_coords
        else:
            self.get_logger().error(f'Projection failed: {response.message}')
            return None
    
    def placement_callback(self, request, response):
        """Handle placement service requests"""

        if len(self.depth_buffer) < 10:
            self.get_logger().error('Waiting for depth images in buffer')
            response.success = False
            response.message = 'Not enough depth images in buffer'
            return response
        
        # Step 1: Validate detections and index
        if not request.detections or len(request.detections) == 0:
            response.success = False
            response.message = 'No object masks found. Critical Error: Segmentation missing'
            return response
        
        if request.detection_index < 0 or request.detection_index >= len(request.detections):
            response.success = False
            response.message = f'Invalid detection index {request.detection_index}. Available: 0-{len(request.detections)-1}'
            return response
        
        # Step 2: Get object mask from detection
        detection = request.detections[request.detection_index]
        
        if detection.mask.width == 0 or detection.mask.height == 0:
            response.success = False
            response.message = f'No mask available for detection at index {request.detection_index}'
            return response
        
        obj_mask = self.get_object_mask_from_msg(detection) 

        # Step 3: Get placements
        depth_image = np.mean([rnp.numpify(img_msg) for img_msg in self.depth_buffer], axis=0)

        self.get_logger().info(f'Finding placements for detection {request.detection_index}')
        res, success = get_placements(depth_image, obj_mask, request.stack)
        
        response.success = success
        response.centroid = Point()
        if success:
            response.centroid.x = res[0][0]
            response.centroid.y = res[0][1]
            response.centroid.z = 0.0
            response.depth = res[1]
            response.message = f'Found valid placement location'
            
            # Visualize placement pixel on color image
            if self.latest_color_image is not None:
                self._visualize_placement(res[0], res[1])
        else:
            response.depth = 0.0
            response.message = f'No valid placements found'
        
        return response
    
    def _visualize_placement(self, placement_pixel, depth):
        """Visualize the placement pixel on the color image"""
        try:
            # Convert color image to numpy array
            color_img = rnp.numpify(self.latest_color_image)
            
            # Log image info for debugging
            self.get_logger().info(f'Image shape: {color_img.shape}, encoding: {self.latest_color_image.encoding}, dtype: {color_img.dtype}')
            self.get_logger().info(f'Image min: {color_img.min()}, max: {color_img.max()}, mean: {color_img.mean():.2f}')
            
            # Ensure it's BGR format for cv2
            if len(color_img.shape) == 2:
                # Grayscale, convert to BGR
                color_img = cv2.cvtColor(color_img, cv2.COLOR_GRAY2BGR)
            elif len(color_img.shape) == 3:
                # Color image - check encoding
                if self.latest_color_image.encoding == 'rgb8':
                    color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
                elif self.latest_color_image.encoding == 'bgr8':
                    # Already BGR, no conversion needed
                    pass
                else:
                    # Try to handle other encodings
                    self.get_logger().warn(f'Unknown encoding: {self.latest_color_image.encoding}, assuming RGB')
                    color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
            
            # Ensure uint8 type
            if color_img.dtype != np.uint8:
                if color_img.max() <= 1.0:
                    color_img = (color_img * 255).astype(np.uint8)
                else:
                    color_img = color_img.astype(np.uint8)
            
            # Check if image is all zeros or very dark
            if color_img.max() == 0:
                self.get_logger().warn('Image is all zeros!')
                return
            elif color_img.mean() < 10:
                self.get_logger().warn(f'Image is very dark (mean: {color_img.mean():.2f})')
            
            # Log final image stats
            self.get_logger().info(f'Final image shape: {color_img.shape}, dtype: {color_img.dtype}, min: {color_img.min()}, max: {color_img.max()}, mean: {color_img.mean():.2f}')
            
            # Make a copy to avoid modifying the original
            vis_img = color_img.copy()
            
            # Get pixel coordinates
            px, py = int(placement_pixel[0]), int(placement_pixel[1])
            
            # Ensure coordinates are within image bounds
            h, w = vis_img.shape[:2]
            px = max(0, min(px, w - 1))
            py = max(0, min(py, h - 1))
            
            # Draw a circle at the placement location
            cv2.circle(vis_img, (px, py), 10, (0, 255, 0), 2)  # Green circle
            cv2.circle(vis_img, (px, py), 2, (0, 0, 255), -1)  # Red center dot
            
            # Draw a crosshair
            cv2.line(vis_img, (px - 15, py), (px + 15, py), (0, 255, 0), 2)
            cv2.line(vis_img, (px, py - 15), (px, py + 15), (0, 255, 0), 2)
            
            # Add text with depth information
            text = f'Placement: ({px}, {py}), Depth: {depth:.3f}m'
            cv2.putText(vis_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (0, 255, 0), 2)
            
            # Save the image to disk
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            save_dir = '/tmp/placement_visualizations'
            os.makedirs(save_dir, exist_ok=True)
            filename = os.path.join(save_dir, f'placement_{timestamp}.png')
            cv2.imwrite(filename, vis_img)
            self.get_logger().info(f'Saved placement visualization to: {filename}')
            
            # Try to display with cv2.imshow
            try:
                window_name = 'Placement Visualization'
                # Create or update the window
                if not self.window_created:
                    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(window_name, 1280, 720)  # Make it larger
                    self.window_created = True
                
                cv2.imshow(window_name, vis_img)
                # Use waitKey with a small delay to ensure window updates
                key = cv2.waitKey(10) & 0xFF
                if key == ord('q'):
                    cv2.destroyWindow(window_name)
                    self.window_created = False
                    self.get_logger().info('Placement visualization window closed by user')
            except Exception as e:
                self.get_logger().warn(f'Could not display image with cv2.imshow: {e}')
                self.get_logger().warn('Image saved to disk instead. Check the log for file path.')
            
        except Exception as e:
            self.get_logger().error(f'Failed to visualize placement: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    
    placement_server = PlacementServer()
    
    try:
        rclpy.spin(placement_server)
    except KeyboardInterrupt:
        pass
    finally:
        placement_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

