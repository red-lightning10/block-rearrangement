# Segmentation Package

Service for detecting and segmenting objects from camera images.

## Service: GetSegmentation

### Request:
- Empty (no input needed, uses buffered images)

### Response:
- `mask` (Image) - Segmentation mask
- `centroid` (Point) - Centroid coordinates (x, y pixel coords)
- `depth` (float32) - Depth at centroid
- `success` (bool) - Operation status
- `message` (string) - Status message

## Behavior

- Subscribes to `/camera/camera/color/image_raw` and `/camera/camera/depth/image_rect_raw`
- Buffers last 10 images (temporal variation handling)
- When service is called, processes all 10 images through segmentation
- Returns mask + centroid + depth

## Usage

```bash
ros2 run segmentation segmentation_server
```

## TODO

- [ ] **Add your segmentation function call** (currently has `pass` placeholder)
- [ ] The segmentation function should take `cv_images` (list of numpy arrays) and return `mask` and `centroid`
- [ ] Replace mock response with actual segmentation results

## Where to Add Your Logic

In `segmentation_server.py`, line ~70:

```python
# TODO: Call your segmentation function here
# mask, centroid = your_segmentation_function(cv_images)

pass  # <-- Replace this with your actual segmentation call
```

