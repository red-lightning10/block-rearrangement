"""
Placement computation utilities for finding valid placement locations.
Handles convolution-based placement validation and cluster validation.
"""
import numpy as np
import cv2
from scipy import signal
from scipy.stats import tmean
from scipy.ndimage import rotate
from .object_utils import get_kernel


def valid_centers(objMask, regionMask, obstaclesMask):
    """
    Computes the centers in the available surface where the object can be placed 
    by convolving the obstacles by the objects mask

    Inputs:
        - objMask: the mask of the object to place
        - regionMask: mask of valid placement regions
        - obstaclesMask: mask of obstacles
        - debug: boolean for visualizing outputs

    Returns:
        - val_mask: centers where object fits in both orientations
        - allVal: all centers where object fits (any orientation)
        - final_centersFirst: centers for first orientation
        - final_centersSecond: centers for second orientation (90 deg rotated)
    """
    kernel = get_kernel(objMask.copy())  # gets the kernel for dilation

    kernel_clean = np.ones((3, 3), np.uint8)  # a kernel that will be used to clean our masks

    processed_mask = signal.convolve2d(obstaclesMask.copy(), kernel, boundary='symm', mode='same')  # convolve the obstacles by the object mask

    centers_first = np.asanyarray(processed_mask, np.uint8)
    val_first = np.asanyarray(centers_first == 0, bool)  # converts mask to boolean of all of the feasible centers after the convolution
    val_centers_first = val_first.astype(np.uint8) * 255  # marks feasible centers as white
    
    val_centers_first = cv2.bitwise_and(regionMask.copy(), val_centers_first)  # combines with the original scene to get rid of false positives

    final_centersFirst = cv2.morphologyEx(val_centers_first, cv2.MORPH_OPEN, kernel_clean)  # gets rid of remaining false positives (lone positive pixels)

    processed_mask_2 = signal.convolve2d(obstaclesMask.copy(), kernel.T, boundary='symm', mode='same')  # convolve the obstacle by the object rotated 90 degrees

    centers_second = np.asanyarray(processed_mask_2, np.uint8)

    val_second = np.asanyarray(centers_second == 0, bool)  # converts mask to boolean of all of the feasible centers after the convolution
    val_centers_second = val_second.astype(np.uint8) * 255  # marks feasible centers as white

    val_centers_second = cv2.bitwise_and(regionMask.copy(), val_centers_second)  # combines with the original scene to get rid of false positives

    final_centersSecond = cv2.morphologyEx(val_centers_second, cv2.MORPH_OPEN, kernel_clean)  # gets rid of remaining false positives (lone positive pixels)

    val_mask = cv2.bitwise_and(final_centersFirst, final_centersSecond)  # centers where the object fits placed in both orientations (as seen in the object mask and rotated 90 degrees)
    allVal = cv2.bitwise_or(final_centersFirst, final_centersSecond)  # all centers where the object fits

    return val_mask, allVal, final_centersFirst, final_centersSecond


def validate_clusters(clusters, objMask):
    """
    Validate clusters for placement by finding the best orientation.
    
    Inputs:
        - clusters: list of cluster depth masks
        - objMask: object mask to place
        - debug: boolean for visualization
    
    Returns:
        - (cx, cy): center coordinates, or (cx, cy), angle if valid orientation found
    """
    processed_object = cv2.erode(objMask.copy(), np.ones((3, 3)), iterations=1)
    processed_object = cv2.dilate(processed_object, np.ones((3, 3)), iterations=1)
    maxClusterDepth = None
    maxClusterBinary = None
    maxHeight = 0

    for cluster in clusters:
        binary_cluster = np.asanyarray(cluster > 0, bool).astype(np.uint8) * 255
        height = tmean(cluster.ravel())
        if height > maxHeight:
            maxHeight = height
            maxClusterDepth = cluster
            maxClusterBinary = binary_cluster
    
    kernel = get_kernel(processed_object.copy())

    cnts, _ = cv2.findContours(maxClusterBinary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    maxClusterContour = max(cnts, key=cv2.contourArea)

    objSum = cv2.countNonZero(kernel)

    (cx, cy), (w, h), rect_angle = cv2.minAreaRect(maxClusterContour)

    hw = round(w/2)
    hh = round(h/2)

    placementObstacleMask = maxClusterBinary[(round(cy) - hw):(round(cy) + hw), (round(cx) - hh):(round(cx) + hh)]

    placementObstacleMask = cv2.erode(placementObstacleMask, np.ones((3, 3)), iterations=1)
    placementObstacleMask = cv2.dilate(placementObstacleMask, np.ones((3, 3)), iterations=1)

    minY, maxY = min(len(placementObstacleMask), len(kernel)), max(len(placementObstacleMask), len(kernel))
    minX, maxX = min(len(placementObstacleMask[0]), len(kernel[0])), max(len(placementObstacleMask[0]), len(kernel[0]))

    a = abs((maxY - minY) // 2)
    aa = abs(maxY - a - minY)

    b = abs((maxX - minX) // 2)
    bb = abs(maxX - b - minX)

    if len(placementObstacleMask) < len(kernel): 
        placementObstacleMask = np.pad(placementObstacleMask, pad_width=((a, aa), (0, 0)), mode='constant', constant_values=0)
    elif len(kernel) < len(placementObstacleMask): 
        kernel = np.pad(kernel, pad_width=((a, aa), (0, 0)), mode='constant', constant_values=0)

    if len(placementObstacleMask[0]) < len(kernel[0]): 
        placementObstacleMask = np.pad(placementObstacleMask, pad_width=((0, 0), (b, bb)), mode='constant', constant_values=0)
    elif len(kernel[0]) < len(placementObstacleMask[0]): 
        kernel = np.pad(kernel, pad_width=((0, 0), (b, bb)), mode='constant', constant_values=0)
    
    kernelClone = kernel.copy()
    for i in range(0, 360, 1):
        matrix = placementObstacleMask * kernelClone
        matSum = cv2.countNonZero(matrix)
        kernelClone = rotate(kernel.copy(), i, reshape=False)

        if matSum >= objSum:
            return (cx, cy), i
        else: 
            kernelClone = rotate(kernel.copy(), i + 1, reshape=False)

    return (cx, cy)


def get_placements(depth_image, objMask, stack):
    """
    Main placement computation function.
    
    Inputs:
        - depth_image (HxW): depth image in mm as numpy array
        - objMask: object mask to place
        - stack: true for stacking on other objects, false for only placing on table
    
    Returns:
        - ((x, y), depth), success: tuple of placement location and depth, and success boolean
    """
    from .clustering import hdbScan
    table_mask, obstacles_mask = hdbScan(depth_image, k=2.0)

    binary_table_mask = np.asanyarray(table_mask > 0, bool).astype(np.uint8) * 255

    if not stack:
        # Convert list of obstacle masks to binary masks and combine them
        obs_mask = np.zeros_like(binary_table_mask)
        if obstacles_mask:  # Check if list is not empty
            for obs_mask_item in obstacles_mask:
                binary_obs = np.asanyarray(obs_mask_item > 0, bool).astype(np.uint8) * 255
                obs_mask = cv2.bitwise_or(obs_mask, binary_obs)
        val_mask, allVal, _, _ = valid_centers(objMask, binary_table_mask, obs_mask)

        # Get all valid placement locations
        valid_locations = np.argwhere(allVal == 255)
        if len(valid_locations) == 0:
            return ((0.0, 0.0), 0.0), False
        
        # Randomly choose one valid location
        placement_idx = np.random.choice(len(valid_locations))
        placement_pixel = valid_locations[placement_idx]  # [y, x]
        # Get depth from original depth_image (in mm), convert to meters
        depth = float(depth_image[placement_pixel[0], placement_pixel[1]]) / 1000.0
        
        # Convert from [y, x] to (x, y) for return, as floats
        placement_xy = (float(placement_pixel[1]), float(placement_pixel[0]))
        return (placement_xy, depth), True

    else:
        if not obstacles_mask:
            return ((0.0, 0.0), 0.0), False
        
        placement_result = validate_clusters(obstacles_mask, objMask)
        
        # Debug: print what we got
        print(f'DEBUG: placement_result type: {type(placement_result)}, value: {placement_result}')
        
        # Handle return value: can be (cx, cy) or ((cx, cy), angle)
        # Check if it's a tuple of 2 elements where the first is also a tuple
        if (isinstance(placement_result, tuple) and 
            len(placement_result) == 2 and 
            isinstance(placement_result[0], tuple) and 
            isinstance(placement_result[1], (int, float))):
            # Returned ((cx, cy), angle)
            placement_pixel = placement_result[0]
            print(f'DEBUG: Extracted placement_pixel from ((cx, cy), angle): {placement_pixel}')
        elif isinstance(placement_result, tuple) and len(placement_result) == 2:
            # Returned (cx, cy) - tuple of 2 floats
            placement_pixel = placement_result
            print(f'DEBUG: Using placement_result directly as (cx, cy): {placement_pixel}')
        else:
            # Unexpected format
            print(f'ERROR: Unexpected return format from validate_clusters: {type(placement_result)}, value: {placement_result}')
            return ((0.0, 0.0), 0.0), False
        
        # Get depth from the original depth_image at the placement location
        # (obstacle masks contain normalized depth, not actual depth)
        if not isinstance(placement_pixel, tuple) or len(placement_pixel) != 2:
            # self.node.get_logger().error(f'Invalid placement_pixel format: {type(placement_pixel)}, value: {placement_pixel}')
            return ((0.0, 0.0), 0.0), False
        
        cx, cy = float(placement_pixel[0]), float(placement_pixel[1])
        # Ensure indices are within bounds
        cy_idx = max(0, min(int(cy), depth_image.shape[0] - 1))
        cx_idx = max(0, min(int(cx), depth_image.shape[1] - 1))
        # depth_image is in mm, convert to meters
        depth = float(depth_image[cy_idx, cx_idx]) / 1000.0
        return ((cx, cy), depth), True


