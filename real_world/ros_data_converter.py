from sensor_msgs.msg import PointCloud2, JointState, Image
from geometry_msgs.msg import PoseStamped
import numpy as np
import open3d as o3d
import cv2
from typing import Dict, Tuple, List, Optional
from loguru import logger
from cv_bridge import CvBridge
from common.space_utils import (ros_pose_to_4x4matrix, matrix4x4_to_pose_6d)
from common.data_models import SensorMessage
from real_world.real_world_transforms import RealWorldTransforms
from ros2_numpy import point_cloud2 as pc2
from common.visualization_utils import visualize_pcd_from_numpy, visualize_rgb_image

class ROS2DataConverter:
    """
    Data converter class that converts ROS2 topic data into Pydantic data models
    """
    def __init__(self,
                 transforms: RealWorldTransforms,
                 depth_camera_point_cloud_topic_names: List[Optional[str]] = [None, None, None],  # external, left wrist, right wrist
                 depth_camera_rgb_topic_names: List[Optional[str]] = [None, None, None],  # external, left wrist, right wrist
                 tactile_camera_rgb_topic_names: List[Optional[str]] = [None, None, None, None],  # left gripper1, left gripper2, right gripper1, right gripper2
                 tactile_camera_marker_topic_names: List[Optional[str]] = [None, None, None, None], # left gripper1, left gripper2, right gripper1, right gripper2
                 debug = True,
                 tactile_camera_rgb_image_shape: Tuple = (352, 288)):
        self.transforms = transforms
        self.debug = debug
        self.depth_camera_point_cloud_topic_names = depth_camera_point_cloud_topic_names
        self.depth_camera_rgb_topic_names = depth_camera_rgb_topic_names
        self.tactile_camera_rgb_topic_names = tactile_camera_rgb_topic_names
        self.tactile_camera_marker_topic_names = tactile_camera_marker_topic_names
        self.bridge = CvBridge()
        self.tactile_camera_rgb_image_shape = tactile_camera_rgb_image_shape

    def visualize_tcp_poses(self, tcp_pose_left_in_world: np.ndarray, tcp_pose_right_in_world: np.ndarray):
        world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

        left_tcp = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        left_tcp.transform(tcp_pose_left_in_world)

        left_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        left_base.transform(self.transforms.left_robot_base_to_world_transform)

        right_tcp = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        right_tcp.transform(tcp_pose_right_in_world)

        right_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        right_base.transform(self.transforms.right_robot_base_to_world_transform)

        o3d.visualization.draw_geometries([world, left_tcp, left_base, right_tcp, right_base])

    def convert_robot_states(self, topic_dict: Dict) -> Tuple[np.ndarray, np.ndarray]:
        # TODO: use adaptive topic name
        tcp_pose_left: PoseStamped = topic_dict['/tcp_pose_left']
        tcp_pose_right: PoseStamped = topic_dict['/tcp_pose_right']

        left_gripper_state: JointState = topic_dict['/left_gripper_state']
        right_gripper_state: JointState = topic_dict['/right_gripper_state']

        # convert TCP pose in robot coordinate to world coordinate
        left_tcp_mat_in_world = (self.transforms.left_robot_base_to_world_transform @
                             ros_pose_to_4x4matrix(tcp_pose_left.pose))
        right_tcp_mat_in_world = (self.transforms.right_robot_base_to_world_transform @
                              ros_pose_to_4x4matrix(tcp_pose_right.pose))

        left_tcp_6d_pose_in_world = matrix4x4_to_pose_6d(left_tcp_mat_in_world)
        right_tcp_6d_pose_in_world = matrix4x4_to_pose_6d(right_tcp_mat_in_world)
        # (6+2) TCP (x, y, z, r, p, y) + gripper (width, force)
        left_robot_state = np.concatenate([left_tcp_6d_pose_in_world,
                                           left_gripper_state.position, left_gripper_state.effort]).astype(
            np.float32)
        right_robot_state = np.concatenate([right_tcp_6d_pose_in_world,
                                            right_gripper_state.position, right_gripper_state.effort]).astype(
            np.float32)

        if self.debug:
            logger.debug(f"Left robot state (6+2) TCP (x, y, z, r, p, y) + gripper (width, force): {left_robot_state}")
            logger.debug(
                f"Right robot state (6+2) TCP (x, y, z, r, p, y) + gripper (width, force): {right_robot_state}")
            self.visualize_tcp_poses(left_tcp_mat_in_world, right_tcp_mat_in_world)

        return left_robot_state, right_robot_state

    def decode_point_cloud(self, msg: PointCloud2) -> np.ndarray:
        # Decode the point cloud data
        cloud_arr = pc2.pointcloud2_to_array(msg)

        # Extract the fields        
        points = np.vstack((cloud_arr['x'], cloud_arr['y'], cloud_arr['z'])).T
        colors = np.vstack((cloud_arr['r'], cloud_arr['g'], cloud_arr['b'])).T / 255.0
        points = np.hstack((points, colors))

        if self.debug:
            visualize_pcd_from_numpy(points)

        return points    
    
    def decode_depth_rgb_image(self, msg: Image) -> np.ndarray:
        # Decode the image from JPEG format
        np_arr = np.frombuffer(msg.data, np.uint8)
        color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if self.debug:
            if color_image is not None:
                # Display the image using OpenCV
                visualize_rgb_image(color_image, "Received Image")
            else:
                logger.debug('Failed to decode image')
        
        return color_image

    def decode_rgb_image(self, msg: Image) -> np.ndarray:
        np_arr = np.frombuffer(msg.data, np.uint8)
        color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        return rgb_image

    def convert_depth_camera(self, topic_dict: Dict) -> \
            Tuple[List[Optional[np.ndarray]], List[Optional[np.ndarray]]]:
        point_cloud_list = []
        for idx, topic_name in enumerate(self.depth_camera_point_cloud_topic_names):
            if topic_name is not None:
                if self.debug:
                    logger.debug(topic_name)
                assert topic_name in topic_dict, f"Topic {topic_name} not found in topic_dict"
                point_cloud_list.append(self.decode_point_cloud(topic_dict[topic_name]))
            else:
                point_cloud_list.append(None)

        rgb_image_list = []
        for idx, topic_name in enumerate(self.depth_camera_rgb_topic_names):
            if topic_name is not None:
                if self.debug:
                    logger.debug(topic_name)
                assert topic_name in topic_dict, f"Topic {topic_name} not found in topic_dict"
                rgb_image_list.append(self.decode_depth_rgb_image(topic_dict[topic_name]))
            else:
                rgb_image_list.append(None)

        return point_cloud_list, rgb_image_list

    def convert_all_data(self, topic_dict: Dict) -> SensorMessage:
        # calculate the lastest timestamp in the topic_dict
        latest_timestamp = max([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                                for msg in topic_dict.values()])

        # TODO: dynamically get running topics, changes in the converter functions
        left_robot_state, right_robot_state = self.convert_robot_states(topic_dict)
        depth_camera_pointcloud_list, depth_camera_rgb_list = self.convert_depth_camera(topic_dict)

        sensor_msg_args = {
        'timestamp': latest_timestamp,
        'leftRobotState': left_robot_state,
        'rightRobotState': right_robot_state,
        }
        
        if depth_camera_pointcloud_list[0] is not None:
            sensor_msg_args['externalCameraPointCloud'] = depth_camera_pointcloud_list[0]
        if depth_camera_rgb_list[0] is not None:
            sensor_msg_args['externalCameraRGB'] = depth_camera_rgb_list[0]
        if depth_camera_pointcloud_list[1] is not None:
            sensor_msg_args['leftWristCameraPointCloud'] = depth_camera_pointcloud_list[1]
        if depth_camera_rgb_list[1] is not None:
            sensor_msg_args['leftWristCameraRGB'] = depth_camera_rgb_list[1]
        if depth_camera_pointcloud_list[2] is not None:
            sensor_msg_args['rightWristCameraPointCloud'] = depth_camera_pointcloud_list[2]
        if depth_camera_rgb_list[2] is not None:
            sensor_msg_args['rightWristCameraRGB'] = depth_camera_rgb_list[2]

        sensor_msg = SensorMessage(**sensor_msg_args)
        
        return sensor_msg
        
    
 

