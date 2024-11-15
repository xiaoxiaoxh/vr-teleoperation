import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import cv2
import numpy as np
import open3d as o3d
import time
from loguru import logger

from third_party.ros2_numpy_humble.ros2_numpy import point_cloud2 as pc2

class RealsenseCameraSubscriber(Node):
    def __init__(self, 
                 camera_serial_number: str = '036422060422',
                 camera_name: str = 'camera_base',
                 debug_depth = False,
                 debug_rgb = False
                 ):
        node_name = f'{camera_name}_subscriber'
        super().__init__(node_name)
        self.depth_subscription = self.create_subscription(
            PointCloud2,
            f'/{camera_name}/depth/points',  
            self.depth_listener_callback,
            10
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            f'/{camera_name}/color/image_raw',
            self.rgb_listener_callback,
            10
        )
        self.camera_name = camera_name
        self.prev_time = time.time()
        self.frame_count = 0
        self.debug_depth = debug_depth
        self.debug_rgb = debug_rgb

    def depth_listener_callback(self, msg):
        self.get_logger().debug(f"Received point cloud with {msg.width * msg.height} points")
        self.get_logger().debug(f"width:{msg.width} height:{msg.height}")
        start_time = time.time()

        cloud_arr = pc2.pointcloud2_to_array(msg)

        # Extract the fields        
        points = np.vstack((cloud_arr['x'], cloud_arr['y'], cloud_arr['z'])).T
        colors = np.vstack((cloud_arr['r'], cloud_arr['g'], cloud_arr['b'])).T
        points = np.hstack((points, colors))

        # Calculate and print frame rate
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.prev_time
        if (elapsed_time >= 1.0):
            frame_rate = self.frame_count / elapsed_time
            self.get_logger().info(f"Frame rate: {frame_rate:.2f} FPS")
            self.prev_time = current_time
            self.frame_count = 0

        end_time = time.time()

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd.colors = o3d.utility.Vector3dVector(points[:, 3:6] / 255.0)
        if self.debug_depth:
            coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
            o3d.visualization.draw_geometries([pcd, coord])

    def rgb_listener_callback(self, msg):
        try:
            # Decode the image from JPEG format
            np_arr = np.frombuffer(msg.data, np.uint8)
            color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if self.debug_rgb:
                if color_image is not None:
                    # Display the image using OpenCV
                    cv2.imshow("Received Image", color_image)
                    cv2.waitKey(1)
                else:
                    logger.debug('Failed to decode image')

        except Exception as e:
            logger.error(f"Error during image decoding: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Realsense Camera Subscriber')
    parser.add_argument('--camera_name', type=str, default='external_camera',
                        help='Name of the realsense camera')
    args = parser.parse_args()
        
    data_subscriber = RealsenseCameraSubscriber(camera_name = args.camera_name, debug_depth = False)
    rclpy.spin(data_subscriber)

    data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()