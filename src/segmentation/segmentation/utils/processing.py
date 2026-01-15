import numpy as np
import cv2
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from interfaces.msg import Detection
import ros2_numpy as rnp

def filter_detections_by_confidence(scores, confidence_threshold):
    """
    Filter detections by confidence threshold
    
    Args:
        scores: Detection confidence scores (numpy array)
        boxes: Bounding boxes (numpy array)
        class_ids: Class IDs (numpy array)
        masks: Segmentation masks (numpy array or None)
        confidence_threshold: Minimum confidence threshold (0.0 to 1.0)
    
    Returns:
        high_conf_indices: List of indices for high-confidence detections
    """
    high_conf_indices = [i for i, score in enumerate(scores) if score >= confidence_threshold]
    return high_conf_indices


def get_orientation_from_mask(mask, centroid):
    """
    Get the orientation of a segmentation mask
    
    Args:
        mask: Binary mask (numpy array, uint8, 0-255)
        centroid: [x, y] centroid coordinates
    
    Returns:
        orientation: Angle in degrees (0-360)
        success: Boolean indicating if orientation was computed successfully
    """
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    if contours:
        closest_point = None
        min_distance = float('inf')
        
        for contour in contours:
            for point in contour:
                x, y = point[0]
                dist = np.sqrt((x - centroid[0])**2 + (y - centroid[1])**2)
                
                if dist < min_distance:
                    min_distance = dist
                    closest_point = (x, y)

        if closest_point:
            x1, y1 = centroid[0], centroid[1]
            x2, y2 = closest_point

            dx = x2 - x1
            dy = y2 - y1

            angle_rad = np.arctan2(-dy, dx)
            angle_deg = np.degrees(angle_rad)
            angle_deg = (angle_deg + 360) % 360

            return angle_deg, True
    
    # Failed to get orientation
    return 0.0, False


def create_detection_message(box, depth_image, mask, scores, class_ids, idx):
    """
    Create a Detection message from detection data
    
    Args:
        box: Bounding box [x1, y1, x2, y2]
        depth_image: Depth image (numpy array, in millimeters)
        mask: Segmentation mask (numpy array or None)
        scores: Detection scores array
        class_ids: Class IDs array
        idx: Index of current detection
    
    Returns:
        detection: Detection message
        mask: Original mask (or None)
        success: Boolean indicating if detection was created successfully
    """

    x_center = (box[0] + box[2]) / 2.0
    y_center = (box[1] + box[3]) / 2.0

    depth = float(depth_image[int(y_center), int(x_center)]) / 1000.0
    
    orientation = 0.0
    
    if mask is not None:
        try:
            mask_cv = mask.astype(np.uint8) * 255
            orientation, _ = get_orientation_from_mask(mask_cv, [x_center, y_center])
        except Exception:
            orientation = 0.0
    
    # Create Detection message
    detection = Detection()
    detection.centroid = Point(x=float(x_center), y=float(y_center), z=0.0)
    detection.depth = depth
    detection.orientation = orientation
    detection.confidence = float(scores[idx])
    detection.class_id = int(class_ids[idx])
    
    if mask is not None:
        try:
            # guard against non-2D masks
            if len(mask.shape) != 2:
                raise ValueError(f"Mask must be 2D, got shape {mask.shape}")

            if mask.dtype == bool:
                mask_uint8 = (mask.astype(np.uint8) * 255)
            else:
                mask_uint8 = mask.astype(np.uint8)
                if mask_uint8.max() <= 1:
                    mask_uint8 = mask_uint8 * 255
            
            detection.mask = rnp.msgify(Image, mask_uint8, encoding='mono8')
        except Exception:
            detection.mask = Image()
    else:
        detection.mask = Image()
    
    return detection, mask, True


def temporal_filter_images(image_buffer, depth_buffer):
    """
    Apply temporal filtering to image and depth buffers
    
    Args:
        image_buffer: Deque of color image messages
        depth_buffer: Deque of depth image messages
    
    Returns:
        cv_images: List of color images as numpy arrays (RGB)
        temporal_filtered_depth: Averaged depth image
    """

    cv_images = [cv2.cvtColor(rnp.numpify(img_msg), cv2.COLOR_BGR2RGB) for img_msg in image_buffer]
    depth_images = [rnp.numpify(img_msg) for img_msg in depth_buffer]
    
    #TODO: We alraedy enable temporal filtering, can remove this
    temporal_filtered_depth = np.mean(depth_images, axis=0)
    
    return cv_images, temporal_filtered_depth

def add_masks_to_image(image, masks, alpha=0.5):
    """
    Overlay segmentation masks on image with different colors
    
    Args:
        image: Input image (numpy array, RGB or BGR)
        masks: Binary masks array (N, H, W) where N is number of detections
        alpha: Transparency factor (0.0 = fully transparent, 1.0 = fully opaque)
    
    Returns:
        image_with_masks: Image with overlaid masks
    """
    if masks is None or len(masks) == 0:
        return image

    image_with_masks = image.copy()
    
    # Define colors for different masks (BGR format for OpenCV)
    colors = [
        (255, 0, 0),      # Blue
        (0, 255, 0),      # Green
        (0, 165, 255),    # Orange
        (0, 0, 255),      # Red
        (0, 255, 255),    # Yellow
    ]
    
    # Overlay each mask with a different color
    for idx, mask in enumerate(masks):
        color = colors[idx % len(colors)]
        
        # Create colored mask
        colored_mask = np.zeros_like(image_with_masks, dtype=np.uint8)
        colored_mask[mask > 0] = color
        
        # Blend with original image
        mask_region = mask > 0
        image_with_masks[mask_region] = cv2.addWeighted(
            image_with_masks[mask_region], 
            1 - alpha,
            colored_mask[mask_region], 
            alpha, 
            0
        )
    
    return image_with_masks

def save_visualization_images(color_image, depth_image, masks, output_dir='./src/segmentation/images'):
    """
    Save visualization images to disk
    
    Args:
        color_image: Color image (numpy array)
        depth_image: Depth image (numpy array, in millimeters)
        masks: List of masks to overlay (numpy array or None)
        output_dir: Directory to save images
    """
    
    # Save depth image (convert to 8-bit for visualization)
    depth_vis = cv2.convertScaleAbs(depth_image, alpha=0.03)
    cv2.imwrite(f'{output_dir}/depth_image.png', depth_vis)
    
    # Save color image with overlaid masks (only high-confidence detections)
    if masks is not None and len(masks) > 0:
        masked_image = add_masks_to_image(color_image, np.array(masks), alpha=0.4)
        cv2.imwrite(f'{output_dir}/color_image.png', masked_image)
        return len(masks)
    
    return 0

