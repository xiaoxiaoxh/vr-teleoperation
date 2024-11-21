import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from loguru import logger
import transforms3d as t3d
import numpy as np
import open3d as o3d
import time
from real_world.real_world_transforms import RealWorldTransforms
from hydra import initialize, compose
import threading

class BimanualRobotSubscriber(Node):
    def __init__(self, transforms: RealWorldTransforms):
        super().__init__('bimanual_robot_subscriber')
        self.transforms = transforms
        self.tcp_pose_left_subscriber = self.create_subscription(PoseStamped, 'tcp_pose_left', self.tcp_pose_left_callback, 10)
        self.tcp_pose_right_subscriber = self.create_subscription(PoseStamped, 'tcp_pose_right', self.tcp_pose_right_callback, 10)
        self.left_gripper_state_subscriber = self.create_subscription(JointState, 'left_gripper_state', self.left_gripper_state_callback, 10)
        self.right_gripper_state_subscriber = self.create_subscription(JointState, 'right_gripper_state', self.right_gripper_state_callback, 10)

        self.tcp_poses = {
            'left': np.eye(4),
            'right': np.eye(4)
        }

        logger.info("BimanualRobotSubscriber node initialized")

    def tcp_pose_left_callback(self, msg):
        logger.debug(f"Received left tcp pose: {msg}")
        self.tcp_poses['left'] = self.pose_to_matrix(msg.pose)

    def tcp_pose_right_callback(self, msg):
        logger.debug(f"Received right tcp pose: {msg}")
        self.tcp_poses['right'] = self.pose_to_matrix(msg.pose)

    def pose_to_matrix(self, pose):
        mat = np.eye(4)
        quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        rot_mat = t3d.quaternions.quat2mat(quat)
        mat[:3, :3] = rot_mat
        mat[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])
        return mat

    def left_gripper_state_callback(self, msg):
        logger.debug(f"Received left gripper state: {msg}")

    def right_gripper_state_callback(self, msg):
        logger.debug(f"Received right gripper state: {msg}")

    def visualize_tcp_poses(self):
        world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)

        left_tcp_to_world_transform = self.transforms.left_robot_base_to_world_transform @ self.tcp_poses['left']
        left_tcp = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        left_tcp.transform(left_tcp_to_world_transform)
        logger.debug(f"Left TCP pose: {left_tcp_to_world_transform}")

        left_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        left_base.transform(self.transforms.left_robot_base_to_world_transform)
        logger.debug(f"Left base pose: {self.transforms.left_robot_base_to_world_transform}")

        right_tcp_to_world_transform = self.transforms.right_robot_base_to_world_transform @ self.tcp_poses['right']
        right_tcp = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        right_tcp.transform(right_tcp_to_world_transform)
        logger.debug(f"Right TCP pose: {right_tcp_to_world_transform}")

        right_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        right_base.transform(self.transforms.right_robot_base_to_world_transform)
        logger.debug(f"Right base pose: {self.transforms.right_robot_base_to_world_transform}")

        o3d.visualization.draw_geometries([world, left_tcp, left_base, right_tcp, right_base])

    def start_visualization(self):
        while True:
            self.visualize_tcp_poses()
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)

    with initialize(config_path='../../config/task', version_base="1.1"):
        # config is relative to a module
        cfg = compose(config_name="default")
    transforms = RealWorldTransforms(option=cfg.transforms)
    node = BimanualRobotSubscriber(transforms=transforms)

    thread = threading.Thread(target=node.start_visualization, daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
        thread.join()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
