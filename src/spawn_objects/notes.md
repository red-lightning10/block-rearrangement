# 2025-12-21
- Introduced `SpawnObjects` logic class (`include/spawn_objects/spawn_objects.hpp`, `src/spawn_objects.cpp`) to hold spawn/clear logic outside ROS callbacks.
- `spawn_objects_server.cpp` now contains the node + main and delegates to `SpawnObjects`.
- Removed `src/utils.cpp` and `include/spawn_objects/utils.hpp`; constants now live as `kCubeSize` in `spawn_objects.hpp`.
- `CMakeLists.txt` builds `spawn_objects_server` from `spawn_objects_server.cpp` and `spawn_objects.cpp`.
- Detections subscription now uses transient-local QoS to align behavior with "latest available on join."

# 2025-12-22
## Configuration Centralization
- **Created `config/config.yaml`**: Added ROS parameter configuration with `cube_size: 0.0254` and `camera_frame: "camera_color_optical_frame"`.
- **Removed hardcoded constant**: Removed `kCubeSize` from `spawn_objects.hpp`; `cube_size` is now a member variable passed via constructor.
- **Updated `SpawnObjects` constructor**: Added `cube_size` parameter; all cube size references in code now use `cube_size_` member variable.
- **Updated `spawn_objects_server.cpp`**: Declares `cube_size` parameter using `declare_parameter<double>("cube_size")` (no default value - config mandatory) and passes it to `SpawnObjects` constructor.
- **Updated `CMakeLists.txt`**: Added `install(DIRECTORY config/ ...)` to install config directory.

