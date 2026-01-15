
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from collections import deque
from ament_index_python.packages import get_package_share_directory
from interfaces.msg import DetectionArray
from interfaces.srv import GetSegmentation
from segmentation.utils.detect import load_model
from segmentation.utils.processing import (
    filter_detections_by_confidence,
    create_detection_message,
    temporal_filter_images
)
import ros2_numpy as rnp
import numpy as np


class SegmentationServer(Node):
    """Segmentation node - buffers images, processes them, and publishes detections"""
    
    def __init__(self):
        super().__init__('segmentation_server')

        self.buffer_size = 10
        self.confidence_threshold = 0.66
        self.aligned_depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self._aligned_depth_callback,
            self.buffer_size
        )

        self.aligned_color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self._color_callback,
            self.buffer_size
        )

        # Publisher for detections (with latching QoS)
        latching_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.detections_pub = self.create_publisher(
            DetectionArray,
            '/detections',
            latching_qos
        )

        self.image_buffer = deque(maxlen=self.buffer_size)
        self.depth_buffer = deque(maxlen=self.buffer_size)
        
        # Service to trigger processing on demand
        self.process_service = self.create_service(
            GetSegmentation,
            '/get_segmentation',
            self.process_segmentation_callback
        )
        

        # Load Detectron2 model
        self.predictor = None
        self.metadata = None
        self._load_model()
        
        self.get_logger().info('Segmentation Node started')
        self.get_logger().info('Subscribed to camera topics')
        self.get_logger().info('Publishing detections to /detections topic')
        self.get_logger().info('Waiting for images')
    
    def _load_model(self):
        """Load Detectron2 model for cube detection"""

        package_dir = get_package_share_directory('segmentation')
        model_path = os.path.join(package_dir, 'checkpoints', 'model_final.pth')
        
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model not found at {model_path}')
            return
        
        try:
            self.predictor, self.metadata = load_model(model_path)
            self.get_logger().info('Detectron2 model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
    
    def _aligned_depth_callback(self, msg):
        """Callback for depth images"""
        self.depth_buffer.append(msg)
    
    def _color_callback(self, msg):
        """Callback for color images"""
        self.image_buffer.append(msg)
    
    def _process_detections(self):
        """Process buffered images and return detections (shared logic)"""
        
        if len(self.image_buffer) < 10:
            return None, f'Not enough images buffered (have {len(self.image_buffer)}, need 10)'
        
        self.get_logger().info(f'Processing {len(self.image_buffer)} images')
        
        # Apply temporal filtering
        cv_images, temporal_filtered_depth = temporal_filter_images(self.image_buffer, self.depth_buffer)

        if self.predictor is None:
            return None, 'Model not loaded'
        
        try:
            latest_image = cv_images[-1]
            
            # Run prediction
            outputs = self.predictor(latest_image)
            instances = outputs["instances"]
            
            # Get predictions above threshold
            scores = instances.scores.cpu().numpy()
            boxes = instances.pred_boxes.tensor.cpu().numpy()
            class_ids = instances.pred_classes.cpu().numpy()
            masks = instances.get("pred_masks").cpu().numpy() if instances.has("pred_masks") else None
            
            self.get_logger().info(f'Detected {len(scores)} object(s) total')
            
            # Filter by confidence threshold
            high_conf_indices = filter_detections_by_confidence(scores, self.confidence_threshold)
            
            if len(high_conf_indices) == 0:
                self.get_logger().warn(f'No detections above confidence threshold {self.confidence_threshold}')
                return [], f'No objects detected above confidence threshold {self.confidence_threshold}'
            
            self.get_logger().info(f'{len(high_conf_indices)} object(s) above confidence threshold {self.confidence_threshold}')
            
            # Create Detection message for each high-confidence detection
            detections = []
            
            for idx in high_conf_indices:
                mask = masks[idx] if masks is not None else None
                detection, _, success = create_detection_message(box=boxes[idx], depth_image=temporal_filtered_depth, mask=mask, scores=scores, class_ids=class_ids, idx=idx)
                
                if success:
                    detections.append(detection)
            
            return detections, f'Found {len(detections)} object(s)'
            
        except Exception as e:
            error_msg = str(e) if str(e) else f'{type(e).__name__}'
            self.get_logger().error(f'Detection failed: {error_msg}')
            return None, f'Detection error: {error_msg}'
    
    def _publish_detections(self, detections, success, message):
        """Publish detections to topic"""
        msg = DetectionArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'  # Or appropriate frame
        msg.detections = detections
        msg.success = success
        msg.message = message
        
        self.detections_pub.publish(msg)
        self.get_logger().info(f'Published {len(detections)} detection(s) to /detections topic')
    
    def process_segmentation_callback(self, request, response):
        """Service callback to process detections on demand"""
        detections, message = self._process_detections()
        response.detections = detections if detections else []
        response.masks = []
        response.success = detections is not None
        response.message = message
        
        self._publish_detections(detections, response.success, response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    
    server = SegmentationServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

