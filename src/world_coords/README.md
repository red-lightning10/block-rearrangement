# World Coords Package

Service for converting 2D pixel coordinates to 3D world coordinates using camera intrinsics and extrinsics.

## Service: ProjectTo3D

### Request:
- `centroid` (Point) - 2D pixel coordinates (x, y)
- `depth` (float32) - Depth value at centroid
- `camera_info` (CameraInfo) - Camera intrinsics and extrinsics
- `transform` (TransformStamped) - Transform from camera to world frame

### Response:
- `world_coords` (Point) - 3D world coordinates
- `success` (bool) - Operation status
- `message` (string) - Status message

## Usage:

### Start service server:
```bash
ros2 run world_coords world_coords_server
```

Or launch via `pick_system.launch.py` which includes the world_coords_server node.

### Test with client:
```bash
ros2 run world_coords world_coords_client
```

## TODO:
- [ ] Implement actual back-projection using camera intrinsics
- [ ] Apply transform from camera frame to world frame
- [ ] Handle edge cases (invalid depth, out of bounds pixels)
- [ ] Get camera_info from RealSense camera topics

