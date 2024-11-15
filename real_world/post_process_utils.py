import cv2
import numpy as np
import open3d as o3d
from loguru import logger
from typing import Dict
from common.pcd_utils import farthest_point_sampling
from real_world.real_world_transforms import RealWorldTransforms
from common.data_models import SensorMessage, SensorMode
from common.visualization_utils import visualize_pcd_from_numpy, visualize_rgb_image
from common.space_utils import pose_6d_to_4x4matrix, transform_point_cloud

class DataPostProcessingManager:
    CROPPING_PARAMS = {
        'external': {
            'min': np.array([[0.4, -2.0, 0.05]]).T,
            'max': np.array([[1.2, 2.0, 1.0]]).T
        },
        'left_wrist': {
            'min': np.array([[-1.0, -1.0, 0.]]).T,
            'max': np.array([[1.0, 1.0, 0.5]]).T
        },
        'right_wrist': {
            'min': np.array([[-1.0, -1.0, 0.]]).T,
            'max': np.array([[1.0, 1.0, 0.5]]).T
        },
        'merged': {
            'min': np.array([[0.4, -2.0, 0.05]]).T,
            'max': np.array([[1.2, 2.0, 1.0]]).T
        },
    }
    def __init__(self,
                 transforms: RealWorldTransforms,
                 mode: str = 'single_arm_two_realsense',
                 IMAGE_RESIZE_FACTOR: float = 0.5,
                 use_6d_rotation: bool = True,
                 use_merged_pcd: bool = True,
                 debug: bool = False):
        self.transforms = transforms
        self.mode = SensorMode[mode]
        self.use_6d_rotation = use_6d_rotation
        self.use_merged_pcd = use_merged_pcd
        self.RESIZE_FACTOR = IMAGE_RESIZE_FACTOR
        self.debug = debug

    @staticmethod
    def convert_8d_state_to_11d_state(state: np.ndarray) -> np.ndarray:
        """
        Convert 8D state to 11D state
        :param state: np.ndarray (8,), (x, y, z, rx, ry, rz, gripper_width, gripper_force)
        :return: np.ndarray (11,), (x, y, z, rx1, rx2, rx3, ry1, ry2, ry3, gripper_width, gripper_force)
        """
        robot_tcp_rot_6d = pose_6d_to_4x4matrix(state)[:3, :2].T.flatten()
        return np.concatenate((state[:3], robot_tcp_rot_6d, state[-2:]), axis=0)

    def convert_sensor_msg_to_obs_dict(self, sensor_msg: SensorMessage) -> Dict[str, np.ndarray]:
        obs_dict = dict()
        if self.use_6d_rotation:
            left_robot_state_11d = self.convert_8d_state_to_11d_state(sensor_msg.leftRobotState)
            right_robot_state_11d = self.convert_8d_state_to_11d_state(sensor_msg.rightRobotState)
            obs_dict['state'] = np.concatenate((left_robot_state_11d, right_robot_state_11d), axis=0)
            # TODO: support delta action
            left_robot_action_11d = self.convert_8d_state_to_11d_state(sensor_msg.leftRobotAction)
            right_robot_action_11d = self.convert_8d_state_to_11d_state(sensor_msg.rightRobotAction)
            obs_dict['action'] = np.concatenate((left_robot_action_11d, right_robot_action_11d), axis=0)
        else:
            obs_dict['state'] = np.concatenate((sensor_msg.leftRobotState, sensor_msg.rightRobotState), axis=0)
            obs_dict['action'] = np.concatenate((sensor_msg.leftRobotAction, sensor_msg.rightRobotAction), axis=0)
        if self.debug:
            logger.debug(f'state: {obs_dict["state"]}')
            logger.debug(f'action: {obs_dict["action"]}')

        # TODO: make all sensor post-processing in parallel
        external_pcd = sensor_msg.externalCameraPointCloud.astype(np.float32)  # (x, y, z, r, g, b)
        if self.use_merged_pcd:
            obs_dict['external_pcd'] = transform_point_cloud(external_pcd, self.transforms.external_camera_to_world_transform)
        else:
            obs_dict['external_pcd'] = self.post_process_pcd(external_pcd,
                                                           transform_matrix=self.transforms.external_camera_to_world_transform,
                                                           crop_min=self.CROPPING_PARAMS['external']['min'],
                                                           crop_max=self.CROPPING_PARAMS['external']['max']).astype(np.float16)

        obs_dict['external_img'] = self.resize_image_by_factor(sensor_msg.externalCameraRGB, factor=self.RESIZE_FACTOR)
        if self.debug:
            visualize_pcd_from_numpy(obs_dict['external_pcd'])
            logger.debug(f'external_pcd shape: {obs_dict["external_pcd"].shape}')
            visualize_rgb_image(obs_dict['external_img'])
        if self.mode == SensorMode.single_arm_one_realsense:
            if self.use_merged_pcd:
                merged_pcd = obs_dict['external_pcd']
                obs_dict['merged_pcd'] = self.post_process_pcd(merged_pcd,
                                                           transform_matrix=np.eye(4),
                                                           crop_min=self.CROPPING_PARAMS['merged']['min'],
                                                           crop_max=self.CROPPING_PARAMS['merged']['max']).astype(np.float16)
            return obs_dict

        left_wrist_pcd = sensor_msg.leftWristCameraPointCloud.astype(np.float32)  # (x, y, z, r, g, b)
        left_tcp_pose_6d = sensor_msg.leftRobotState[:6]
        left_wrist_camera_to_world_transform = (pose_6d_to_4x4matrix(left_tcp_pose_6d) @
                                           self.transforms.left_wrist_camera_to_left_robot_tcp_transform)
        if self.use_merged_pcd:
            obs_dict['left_wrist_pcd'] = transform_point_cloud(left_wrist_pcd, left_wrist_camera_to_world_transform)
        else:
            obs_dict['left_wrist_pcd'] = self.post_process_pcd(left_wrist_pcd,
                                                    transform_matrix=left_wrist_camera_to_world_transform,
                                                    crop_min=self.CROPPING_PARAMS['left_wrist']['min'],
                                                    crop_max=self.CROPPING_PARAMS['left_wrist']['max']).astype(np.float16)
        obs_dict['left_wrist_img'] = self.resize_image_by_factor(sensor_msg.leftWristCameraRGB, factor=self.RESIZE_FACTOR)
        if self.debug:
            visualize_pcd_from_numpy(obs_dict['left_wrist_pcd'])
            logger.debug(f'left_wrist_pcd shape: {obs_dict["left_wrist_pcd"].shape}')
            visualize_rgb_image(obs_dict['left_wrist_img'])
        if self.mode == SensorMode.single_arm_two_realsense:
            if self.use_merged_pcd:
                merged_pcd = np.concatenate((obs_dict['external_pcd'], obs_dict['left_wrist_pcd']), axis=0)
                obs_dict['merged_pcd'] = self.post_process_pcd(merged_pcd,
                                                           transform_matrix=np.eye(4),
                                                           crop_min=self.CROPPING_PARAMS['merged']['min'],
                                                           crop_max=self.CROPPING_PARAMS['merged']['max']).astype(np.float16)
            return obs_dict

        right_wrist_pcd = sensor_msg.rightWristCameraPointCloud.astype(np.float32)  # (x, y, z, r, g, b)
        right_tcp_pose_6d = sensor_msg.rightRobotState[:6]
        right_wrist_camera_to_world_transform = (pose_6d_to_4x4matrix(right_tcp_pose_6d) @
                                           self.transforms.right_wrist_camera_to_right_robot_tcp_transform)
        if self.use_merged_pcd:
            obs_dict['right_wrist_pcd'] = transform_point_cloud(right_wrist_pcd, right_wrist_camera_to_world_transform)
        else:
            obs_dict['right_wrist_pcd'] = self.post_process_pcd(right_wrist_pcd,
                                                            transform_matrix=right_wrist_camera_to_world_transform,
                                                            crop_min=self.CROPPING_PARAMS['right_wrist']['min'],
                                                            crop_max=self.CROPPING_PARAMS['right_wrist']['max']).astype(np.float16)
        obs_dict['right_wrist_img'] = self.resize_image_by_factor(sensor_msg.rightWristCameraRGB, factor=self.RESIZE_FACTOR)

        if self.debug:
            visualize_pcd_from_numpy(obs_dict['right_wrist_pcd'])
            logger.debug(f'right_wrist_pcd shape: {obs_dict["right_wrist_pcd"].shape}')
            visualize_rgb_image(obs_dict['right_wrist_img'])
        if self.mode == SensorMode.dual_arm_two_realsense:
            if self.use_merged_pcd:
                merged_pcd = np.concatenate((obs_dict['external_pcd'], obs_dict['left_wrist_pcd'], obs_dict['right_wrist_pcd']), axis=0)
                obs_dict['merged_pcd'] = self.post_process_pcd(merged_pcd,
                                                           transform_matrix=np.eye(4),
                                                           crop_min=self.CROPPING_PARAMS['merged']['min'],
                                                           crop_max=self.CROPPING_PARAMS['merged']['max']).astype(np.float16)
            return obs_dict
        else:
            raise NotImplementedError

    @staticmethod
    def resize_image_by_factor(image: np.ndarray, factor: float = 0.5) -> np.ndarray:
        return cv2.resize(image, (0, 0), fx=factor, fy=factor)

    @staticmethod
    def post_process_pcd(points: np.ndarray,
                         transform_matrix: np.ndarray = np.eye(4),
                         crop_min: np.ndarray = np.array([[-1.0, -1.0, -1.0]]).T,
                         crop_max: np.ndarray = np.array([[1.0, 1.0, 1.0]]).T,
                         random_sample_num_points: int = 2048,
                         fps_num_points: int = 1024,
                         remove_outliers: bool = True) -> np.ndarray:
        """
        Post process point cloud data by transforming, cropping, random down-sampling, removing outliers and farthest-point-sampling
        :param points: np.ndarray (N, 6) (x, y, z, r, g, b)
        :param transform_matrix: np.ndarray (4, 4), transformation matrix
        :param crop_min: np.ndarray (3, 1), crop min
        :param crop_max: np.ndarray (3, 1), crop max
        :param random_sample_num_points: int, number of points to randomly sample
        :param fps_num_points: int, number of points to farthest-point-sample
        :param remove_outliers: bool, whether to remove outliers
        :return: np.ndarray (num_points, 6) (x, y, z, r, g, b)
        """
        # 1. transform the point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd.colors = o3d.utility.Vector3dVector(points[:, 3:6])
        pcd.transform(transform_matrix)

        # 2. crop the point cloud
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=crop_min, max_bound=crop_max)
        cropped_pcd = pcd.crop(bbox)

        # 3. random select fixed number of points
        pts_xyz = np.asarray(cropped_pcd.points)
        if pts_xyz.shape[0] > random_sample_num_points:
            all_idxs = np.arange(pts_xyz.shape[0])
            rs = np.random.RandomState()
            selected_idxs = rs.choice(all_idxs, size=random_sample_num_points, replace=False)
            pc_xyz_slim = pts_xyz[selected_idxs, :]
            pc_rgb_slim = np.asarray(cropped_pcd.colors)[selected_idxs, :]
            cropped_pcd.points = o3d.utility.Vector3dVector(pc_xyz_slim)
            cropped_pcd.colors = o3d.utility.Vector3dVector(pc_rgb_slim)

        # 4. remove outliers
        if remove_outliers:
            valid_pcd = cropped_pcd
            # Apply statistical outlier removal
            # nb_neighbors: Number of neighbors to analyze for each point
            # std_ratio: Standard deviation multiplier
            cl, ind = valid_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=3.0)
            valid_pcd = valid_pcd.select_by_index(ind)
        else:
            valid_pcd = cropped_pcd

        # Convert to numpy array
        points_xyz = np.asarray(valid_pcd.points)
        if points_xyz.shape[0] == 0:
            valid_pcd = cropped_pcd
            points_xyz = np.asarray(valid_pcd.points)
        points_rgb = np.asarray(valid_pcd.colors)
        point_cloud = np.concatenate((points_xyz, points_rgb), axis=1)

        # 5. farthest point sampling
        point_cloud_xyz, indices = farthest_point_sampling(point_cloud[:, :3], num_points=fps_num_points)
        point_cloud_rgb = point_cloud[indices, 3:6][0]
        point_cloud = np.concatenate((point_cloud_xyz, point_cloud_rgb), axis=1)
        return point_cloud