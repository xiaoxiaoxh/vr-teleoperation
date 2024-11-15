import socket
import numpy as np
import rclpy
import time
import bson
import json
import requests
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
from real_world.real_world_transforms import RealWorldTransforms
from common.data_models import BimanualRobotStates
from loguru import logger

class BimanualRobotPublisher(Node):
    """
    ROS 2 node that publishes gripper states and tcp poses from the bi-manual robot.
    """
    def __init__(self,
                 robot_server_ip: str,
                 robot_server_port: int,
                 transforms: RealWorldTransforms,
                 vr_server_ip: str = '127.0.0.1',
                 vr_server_port: int = 10001,
                 fps: int = 120,
                 debug: bool = False):
        super().__init__('bimanual_robot_publisher')
        self.robot_server_ip = robot_server_ip
        self.robot_server_port = robot_server_port
        # Initialize the real world transforms
        self.transforms = transforms

        self.vr_server_ip = vr_server_ip
        self.vr_server_port = vr_server_port
        self.fps = fps
        self.time_interval = 1 / fps

        # Publishers for TCP poses
        self.tcp_pose_left_publisher = self.create_publisher(PoseStamped, 'tcp_pose_left', 10)
        self.tcp_pose_right_publisher = self.create_publisher(PoseStamped, 'tcp_pose_right', 10)

        # Publishers for gripper states
        self.left_gripper_publisher = self.create_publisher(JointState, 'left_gripper_state', 10)
        self.right_gripper_publisher = self.create_publisher(JointState, 'right_gripper_state', 10)

        # Timer to publish at a fixed rate
        self.timer = self.create_timer(1 / fps, self.timer_callback)
        # Create a session with robot server
        self.session = requests.session()
        # Create a socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # FPS counter
        self.prev_time = time.time()
        self.frame_count = 0
        logger.info("BimanualRobotPublisher node initialized")

        self.debug = debug

    def send_robot_msg(self, robot_states: BimanualRobotStates):

        left_tcp_pose = np.array(robot_states.leftRobotTCP)
        left_tcp_pose_7d_in_unity = self.transforms.robot_frame2unity(left_tcp_pose, left=True)
        right_tcp_pose = np.array(robot_states.rightRobotTCP)
        right_tcp_pose_7d_in_unity = self.transforms.robot_frame2unity(right_tcp_pose, left=False)

        robot_states_in_unity_dict = BimanualRobotStates(
            leftGripperState=robot_states.leftGripperState,
            rightGripperState=robot_states.rightGripperState,
            leftRobotTCP=left_tcp_pose_7d_in_unity,
            rightRobotTCP=right_tcp_pose_7d_in_unity
        ).model_dump()
        if self.debug:
            with open(f'robot_states.json', 'w') as json_file:
                json.dump(robot_states_in_unity_dict, json_file)

        packed_data = bson.dumps(robot_states_in_unity_dict)
        if self.debug:
            logger.debug(f"Sending robot states to VR server: {robot_states_in_unity_dict}")
        self.socket.sendto(packed_data, (self.vr_server_ip, self.vr_server_port))

    def send_command(self, endpoint: str, data: dict = None):
        url = f"http://{self.robot_server_ip}:{self.robot_server_port}{endpoint}"
        if 'get' in endpoint:
            response = self.session.get(url)
        else:
            response = self.session.post(url, json=data)
        response.raise_for_status()  # Raise an error for bad responses
        return response.json()

    def timer_callback(self):
        # this step has 0.5ms - 1ms latency
        robot_states = BimanualRobotStates.model_validate(self.send_command(f'/get_current_robot_states'))
        timestamp = self.get_clock().now().to_msg()

        self.send_robot_msg(robot_states)

        # Create and publish left gripper state
        left_gripper_state = JointState()
        left_gripper_state.header = Header()
        left_gripper_state.header.stamp = timestamp
        left_gripper_state.name = ['left_gripper']
        left_gripper_state.position = [robot_states.leftGripperState[0]]  # Example width in meters
        left_gripper_state.effort = [robot_states.leftGripperState[1]]  # Example force in Newtons
        self.left_gripper_publisher.publish(left_gripper_state)

        # Create and publish right gripper state
        right_gripper_state = JointState()
        right_gripper_state.header = Header()
        right_gripper_state.header.stamp = timestamp
        right_gripper_state.name = ['right_gripper']
        right_gripper_state.position = [robot_states.rightGripperState[0]]  # Example width in meters
        right_gripper_state.effort = [robot_states.rightGripperState[1]]  # Example force in Newtons
        self.right_gripper_publisher.publish(right_gripper_state)

        # Create and publish TCP pose messages for left arm
        tcp_pose_left_msg = PoseStamped()

        tcp_pose_left_msg.header = Header()
        tcp_pose_left_msg.header.stamp = timestamp
        tcp_pose_left_msg.header.frame_id = 'tcp_left'

        # robot_states.leftRobotTCP (x, y, z, qw, qx, qy, qz)
        tcp_pose_left_msg.pose.position = Point(x=robot_states.leftRobotTCP[0],
                                                y=robot_states.leftRobotTCP[1],
                                                z=robot_states.leftRobotTCP[2])
        tcp_pose_left_msg.pose.orientation.w = robot_states.leftRobotTCP[3]
        tcp_pose_left_msg.pose.orientation.x = robot_states.leftRobotTCP[4]
        tcp_pose_left_msg.pose.orientation.y = robot_states.leftRobotTCP[5]
        tcp_pose_left_msg.pose.orientation.z = robot_states.leftRobotTCP[6]
        self.tcp_pose_left_publisher.publish(tcp_pose_left_msg)

        # Create and publish TCP pose messages for right arm
        tcp_pose_right_msg = PoseStamped()
        tcp_pose_right_msg.header = Header()
        tcp_pose_right_msg.header.stamp = timestamp
        tcp_pose_right_msg.header.frame_id = 'tcp_right'

        # robot_states.rightRobotTCP (x, y, z, qw, qx, qy, qz)
        tcp_pose_right_msg.pose.position = Point(x=robot_states.rightRobotTCP[0],
                                                 y=robot_states.rightRobotTCP[1],
                                                 z=robot_states.rightRobotTCP[2])
        tcp_pose_right_msg.pose.orientation.w = robot_states.rightRobotTCP[3]
        tcp_pose_right_msg.pose.orientation.x = robot_states.rightRobotTCP[4]
        tcp_pose_right_msg.pose.orientation.y = robot_states.rightRobotTCP[5]
        tcp_pose_right_msg.pose.orientation.z = robot_states.rightRobotTCP[6]
        self.tcp_pose_right_publisher.publish(tcp_pose_right_msg)

        # calculate fps
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.prev_time
        if elapsed_time >= 1.0:
            frame_rate = self.frame_count / elapsed_time
            logger.debug(f"Frame rate: {frame_rate:.2f} FPS")
            self.prev_time = current_time
            self.frame_count = 0



def main(args=None):
    rclpy.init(args=args)

    from hydra import initialize, compose
    from hydra.utils import instantiate

    with initialize(config_path='../../../config', version_base="1.3"):
        # config is relative to a module
        cfg = compose(config_name="default")

    robot_server = instantiate(cfg.robot_server)
    transforms = RealWorldTransforms(option=cfg.transforms)

    node = BimanualRobotPublisher(robot_server, transforms=transforms, **cfg.publisher.robot_publisher)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()