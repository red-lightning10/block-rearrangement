# Placement Pipeline Utils

This directory contains utility functions for object placement on surfaces, organized into logical modules for use in a ROS package.

## Module Structure

### 1. `depth_processing.py`
**Purpose**: Depth map normalization and preprocessing

**Functions**:
- `normalize_depth(depth_map, k=0.1, debug=False)`: Normalizes depth maps into height contrast maps

**Dependencies**: numpy, cv2, scipy.stats

---

### 2. `clustering.py`
**Purpose**: 3D point clustering to separate table surfaces from obstacles

**Functions**:
- `hdbScan(depth_map, k=0.1, debug=False)`: Clusters scene using HDBSCAN to separate table from obstacles
  - **Input**: depth_map (numpy array in mm)
  - **Returns**: table_mask, obstacles_mask (list)

**Dependencies**: numpy, cv2, hdbscan, depth_processing

---

### 3. `scene_modeling.py`
**Purpose**: Plane fitting (RANSAC) and region segmentation for placement areas

**Functions**:
- `build_scene_model(color_image, scene_depth, camera_intrinsics, debug=False)`: Fits dominant plane and builds scene model
  - **Input**: color_image (numpy array), scene_depth (numpy array in mm), camera_intrinsics (dict)
  - **Returns**: scene dictionary with plane, masks, regions, etc.
- `visualize_scene(scene, color_bgr, win_name="Scene Debug")`: Visualizes scene model

**Dependencies**: numpy, cv2

---

### 4. `object_utils.py`
**Purpose**: Object mask extraction and kernel generation

**Functions**:
- `get_points(mask)`: Gets min/max points of largest contour
- `get_kernel(objectMask, debug=False)`: Creates centered kernel from object mask
- `capture_object_mask(color_image, depth_image, background_gray, background_depth, debug=False)`: 
  Extracts object mask using background subtraction and depth differencing
  - **Input**: All numpy arrays (no RealSense frames)

**Dependencies**: numpy, cv2

---

### 5. `placement.py`
**Purpose**: Placement computation and validation

**Functions**:
- `valid_centers(objMask, regionMask, obstaclesMask, debug=False)`: Computes valid placement centers via convolution
- `validate_clusters(clusters, objMask, debug=False)`: Validates clusters for placement with orientation
- `get_placements(color_image, depth_image, objMask, camera_intrinsics, mode=3, debug=False)`: 
  Main placement computation function
  - **Input**: All numpy arrays, camera_intrinsics dict
  - **Modes**: 1=valid centers only, 2=validate clusters, 3=full pipeline

**Dependencies**: numpy, cv2, scipy, object_utils, clustering

---

## Usage Example

```python
import utils

# Camera intrinsics (from your ROS camera info)
camera_intrinsics = {
    'fx': 615.0,
    'fy': 615.0,
    'ppx': 320.0,
    'ppy': 240.0
}

# Get images as numpy arrays from ROS messages
color_image = # ... your RGB/BGR numpy array
depth_image = # ... your depth numpy array in mm

# Build scene model
scene = utils.build_scene_model(color_image, depth_image, camera_intrinsics, debug=False)

# Get object mask (if you have background)
obj_mask = utils.capture_object_mask(color_image, depth_image, bg_gray, bg_depth, debug=False)

# Find valid placements
val_mask, allVal, first, second = utils.valid_centers(
    obj_mask, 
    scene['region_masks'], 
    scene['obstacle_mask'], 
    debug=False
)
```

## Key Changes from Original

1. **Removed RealSense dependencies**: All functions now work with numpy arrays
2. **Camera intrinsics as parameter**: Passed as dict instead of extracted from frames
3. **No alignment/hole filling**: Assumes already done in ROS pipeline
4. **Modular structure**: Functions organized by purpose for easier maintenance

## Notes

- All depth values are expected in **millimeters (mm)**
- Camera intrinsics should be provided as a dictionary with keys: `'fx'`, `'fy'`, `'ppx'`, `'ppy'`
- Color images can be BGR or RGB (specify in function calls)
- Background subtraction functions expect grayscale background images

