import json
import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformListener, Buffer
from scipy.spatial.transform import Rotation as R
from interfaces.srv import ProjectTo3D


class WorldCoordsServer(Node):
    """Service server for 3D projection and frame transformation"""
    
    def __init__(self):
        super().__init__('world_coords_server')
        
        self.buffer_size = 10
        self.camera_intrinsics = None
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            self.buffer_size
        )

        # Transforms to convert points measured in camera frame to robot base frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load camera-to-end-effector transform stored after eye-in-hand calibration
        config_path = os.path.join(get_package_share_directory('world_coords'), 
                                   'config', 
                                   'calibration_result.json')
        
        with open(config_path, 'r') as f:
            self.extrinsics = json.load(f)
            self.camera_serial = list(self.extrinsics.keys())[0]
        
        # serial number used as key to get the calibration result of specified camera
        cam_transform = self.extrinsics[self.camera_serial]
        self.get_logger().info(f'Camera transform wrt end effector: {cam_transform}')

        self.T_cam_ee = np.array([
            [float(cam_transform['0']['0']), float(cam_transform['0']['1']), 
             float(cam_transform['0']['2']), float(cam_transform['0']['3'])],
            [float(cam_transform['1']['0']), float(cam_transform['1']['1']), 
             float(cam_transform['1']['2']), float(cam_transform['1']['3'])],
            [float(cam_transform['2']['0']), float(cam_transform['2']['1']), 
             float(cam_transform['2']['2']), float(cam_transform['2']['3'])],
            [0.0, 0.0, 0.0, 1.0],
        ])

        self.project_service = self.create_service(
            ProjectTo3D,
            'project_to_3d',
            self.project_callback
        )
        
        self.get_logger().info('World Coords Server started')

    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo topic (make note of namespace topic if multiple cameras)"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = np.array([
                [msg.k[0], msg.k[1], msg.k[2]],
                [msg.k[3], msg.k[4], msg.k[5]],
                [msg.k[6], msg.k[7], msg.k[8]]
            ])
            
            self.get_logger().info(f'Loaded camera intrinsics: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}')

    def project_callback(self, request, response):
        """Project 2D pixel values to 3D world coordinates in base frame"""
        
        self.get_logger().info('Received projection request')
        
        # Check if intrinsics are loaded
        if self.camera_intrinsics is None:
            response.success = False
            response.message = 'Camera intrinsics not loaded yet'
            return response
        
        # Get centroid (x, y) from request
        u = request.centroid.x
        v = request.centroid.y
        depth = request.depth
        
        if depth is None or depth <= 0:
            response.success = False
            response.message = 'Failed to get valid depth value'
            return response

        try:
            # Hardcoded the prefix because I assume we only use UR10 in our lab setup
            ee_to_base = self.tf_buffer.lookup_transform('ur10_base', 'ur10_tool0', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            t = ee_to_base.transform.translation
            R_quat = ee_to_base.transform.rotation

            self.get_logger().info(f'Transform translation: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})')
            self.get_logger().info(f'Transform rotation: ({R_quat.x:.3f}, {R_quat.y:.3f}, {R_quat.z:.3f}, {R_quat.w:.3f})')

            T_ee_base = np.eye(4)
            T_ee_base[:3, :3] = R.from_quat([R_quat.x, R_quat.y, R_quat.z, R_quat.w]).as_matrix()
            T_ee_base[:3, 3] = [t.x, t.y, t.z]

        except Exception as e:
            self.get_logger().warn(f'Could not transform from ur10_tool0 to ur10_base. Error: {e}')
            response.success = False
            response.message = f'Failed to get end effector pose: {str(e)}'
            return response

        # Convert pixel to camera coordinates (pinhole camera model assumption)
        x_cam = (u - self.camera_intrinsics[0, 2]) * depth / self.camera_intrinsics[0, 0]
        y_cam = (v - self.camera_intrinsics[1, 2]) * depth / self.camera_intrinsics[1, 1]
        z_cam = depth
        
        p_cam = np.array([x_cam, y_cam, z_cam, 1.0])
        p_base = T_ee_base @ self.T_cam_ee @ p_cam
        
        response.world_coords = Point(
            x=float(p_base[0]),
            y=float(p_base[1]),
            z=float(p_base[2])
        )
        
        response.success = True
        response.message = 'Projection successful'
        self.get_logger().info(f'Projected to 3D: ({p_base[0]:.3f}, {p_base[1]:.3f}, {p_base[2]:.3f})')
        return response


def main(args=None):
    rclpy.init(args=args)
    server = WorldCoordsServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
