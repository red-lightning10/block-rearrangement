# Detection.msg - Single object detection result

geometry_msgs/Point centroid    # 2D centroid (x, y in pixels, z unused)
float32 depth                   # Depth at centroid in meters
float32 orientation             # Orientation in degrees
float32 confidence              # Detection confidence score (0.0 to 1.0)
int32 class_id                  # Object class ID (0=blue_cube, 1=green_cube, etc.)
sensor_msgs/Image mask          # Segmentation mask of the detected object

