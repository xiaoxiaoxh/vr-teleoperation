import threading
import time
from fastapi import FastAPI
import uvicorn
import requests
import numpy as np
import transforms3d as t3d
from loguru import logger
from common.ring_buffer import RingBuffer
from common.precise_sleep import precise_sleep
from typing import List
from real_world.real_world_transforms import RealWorldTransforms
from common.data_models import UnityMes, BimanualRobotStates

class TeleopServer:
    gripper_interval_count: int = 0
    last_gripper_width_target: List[float] = [0.1, 0.1]
    left_homing_state: bool = False
    left_tracking_state: bool = False
    left_start_real_tcp: np.ndarray = None  # (x, y, z, qw, qx, qy, qz)
    left_start_unity_tcp: np.ndarray = None  # (x, y, z, qw, qx, qy, qz)
    right_homing_state: bool = False
    right_tracking_state: bool = False
    right_start_real_tcp: np.ndarray = None  # (x, y, z, qw, qx, qy, qz)
    right_start_unity_tcp: np.ndarray = None  # (x, y, z, qw, qx, qy, qz)

    def __init__(self,
                 robot_server_ip: str,
                 robot_server_port: int,
                 transforms: RealWorldTransforms,
                 host_ip: str = '192.168.2.187',
                 port: int = 8082,
                 fps: int = 120,
                 # gripper control parameters
                 use_force_control_for_gripper: bool = True,
                 max_gripper_width: float = 0.1,
                 min_gripper_width=0.01,
                 grasp_force=5.0,
                 gripper_control_time_interval=60,
                 gripper_control_width_precision=0.02,
                 ):
        self.robot_server_ip = robot_server_ip
        self.robot_server_port = robot_server_port
        self.host_ip = host_ip
        self.port = port
        self.fps = fps
        self.control_cycle_time = 1 / fps
        self.transforms = transforms
        # gripper control parameters
        self.use_force_control_for_gripper = use_force_control_for_gripper
        self.max_gripper_width = max_gripper_width
        self.min_gripper_width = min_gripper_width
        self.grasp_force = grasp_force
        self.gripper_control_time_interval = gripper_control_time_interval
        self.gripper_control_width_precision = gripper_control_width_precision

        self.msg_buffer = RingBuffer(size=1024, fps=fps)
        self.latest_timestamp = 0.
        # Initialize the session
        self.session = requests.session()
        # Initialize the FastAPI server
        self.app = FastAPI()
        self.setup_routes()

    def setup_routes(self):
        @self.app.post('/unity')
        async def unity(mes: UnityMes):
            self.msg_buffer.push(mes)
            return {'status': 'ok'}

    def run(self):
        teleop_thread = threading.Thread(target=self.process_cmd, daemon=True)
        try:
            teleop_thread.start()
            logger.info("Start Fast-API Tele-operation Server!")
            uvicorn.run(self.app, host=self.host_ip, port=self.port)
            teleop_thread.join()
        except Exception as e:
            logger.exception(e)
            raise e

    def send_command(self, endpoint: str, data: dict = None):
        url = f"http://{self.robot_server_ip}:{self.robot_server_port}{endpoint}"
        if 'get' in endpoint:
            response = self.session.get(url)
        else:
            if 'move' in endpoint:
                # low-level control commands
                try:
                    response = self.session.post(url, json=data, timeout=0.001)
                except requests.exceptions.ReadTimeout:
                    # Ignore the timeout error for low-level control commands to reduce latency
                    # TODO: use a more robust way to handle the timeout error
                    response = None
            else:
                response = self.session.post(url, json=data)
        if response is not None:
            response.raise_for_status()  # Raise an error for bad responses
            return response.json()
        else:
            return dict()

    def set_start_tcp(self, robot_tcp: np.ndarray, unity_tcp: np.ndarray, is_left: bool):
        # record the start position of the robot, and the start position of the unity
        if is_left:
            self.left_start_real_tcp = robot_tcp
            self.left_start_unity_tcp = unity_tcp
            self.left_tracking_state = True
        else:
            self.right_start_real_tcp = robot_tcp
            self.right_start_unity_tcp = unity_tcp
            self.right_tracking_state = True

    def calc_relative_target(self, pos_from_unity: np.ndarray, is_left: bool) -> np.ndarray:
        if is_left:
            start_real_tcp = self.left_start_real_tcp
            start_unity_tcp = self.left_start_unity_tcp
        else:
            start_real_tcp = self.right_start_real_tcp
            start_unity_tcp = self.right_start_unity_tcp
        # unity中相对于记录的unity下起始位置的位移量
        # 加到记录的真实起始位置
        target = np.zeros(7)
        target[:3] = pos_from_unity[:3] - start_unity_tcp[:3] + start_real_tcp[:3]
        target_rot_mat = t3d.quaternions.quat2mat(pos_from_unity[3:]) \
                         @ np.linalg.inv(t3d.quaternions.quat2mat(start_unity_tcp[3:])) \
                         @ t3d.quaternions.quat2mat(start_real_tcp[3:])
        target[3:] = t3d.quaternions.mat2quat(target_rot_mat).tolist()

        return target

    def process_cmd(self):
        while True:
            start_time = time.time()
            try:
                mes, _, _ = self.msg_buffer.peek()
                if mes is None and not (self.left_homing_state or self.right_homing_state):
                    precise_sleep(0.002)
                    continue

            except Exception as e:
                logger.exception(e)

            if self.left_homing_state or self.right_homing_state:
                continue

            if mes.rightHand.cmd == 3 or mes.leftHand.cmd == 3:
                self.left_tracking_state = False
                self.left_homing_state = True
                self.right_tracking_state = False
                self.right_homing_state = True
                # wait until going home is finished
                # TODO: check whether the response will be timed out
                response = self.send_command('/birobot_go_home', {})
                if 'message' in response:
                    logger.info(response['message'])
                self.left_homing_state = False
                self.right_homing_state = False
                continue

            max_gripper_width = self.max_gripper_width
            min_gripper_width = self.min_gripper_width
            grasp_force = self.grasp_force
            gripper_control_width_precision = self.gripper_control_width_precision
            gripper_control_time_interval = self.gripper_control_time_interval
            valid_gripper_interval = max_gripper_width - min_gripper_width
            left_gripper_width_target = max_gripper_width - mes.leftHand.squeeze * valid_gripper_interval
            right_gripper_width_target = max_gripper_width - mes.rightHand.squeeze * valid_gripper_interval
            robot_states = BimanualRobotStates.model_validate(self.send_command('/get_current_robot_states'))
            left_tcp = np.array(robot_states.leftRobotTCP)
            right_tcp = np.array(robot_states.rightRobotTCP)

            if self.gripper_interval_count == 0 and (self.left_tracking_state or self.right_tracking_state):
                # Clear fault in a fixed interval
                # TODO: clear fault in a lower frequency
                # self.send_command('/clear_fault', {})

                # Send gripper command every N times to prevent blocking
                left_current_width = robot_states.leftGripperState[0]
                if abs(self.last_gripper_width_target[0] - left_gripper_width_target) >= gripper_control_width_precision:
                    if self.use_force_control_for_gripper and self.last_gripper_width_target[0] > left_gripper_width_target:
                        # try to close gripper with pure force control
                        logger.debug(f"left gripper moving from {left_current_width} to target: {left_gripper_width_target} "
                                     f"with force {grasp_force}")
                        self.send_command('/move_gripper_force/left', {
                            'force_limit': grasp_force
                        })
                    else:
                        # open gripper with position control
                        logger.debug(f"left gripper moving from {left_current_width} to target: {left_gripper_width_target}")
                        self.send_command('/move_gripper/left', {
                            'width': left_gripper_width_target,
                            'velocity': 10.0,
                            'force_limit': grasp_force
                        })
                    self.last_gripper_width_target[0] = left_gripper_width_target

                right_current_width = robot_states.rightGripperState[0]
                if abs(self.last_gripper_width_target[1] - right_gripper_width_target) >= gripper_control_width_precision:
                    if self.use_force_control_for_gripper and self.last_gripper_width_target[1] > right_gripper_width_target:
                        # try to close gripper with pure force control
                        logger.debug(f"right gripper moving from {right_current_width} to target: {right_gripper_width_target} "
                                     f"with force {grasp_force}")
                        self.send_command('/move_gripper_force/right', {
                            'force_limit': grasp_force
                        })
                    else:
                        # open gripper with position control
                        logger.debug(f"right gripper moving from {right_current_width} to target: {right_gripper_width_target}")
                        self.send_command('/move_gripper/right', {
                            'width': right_gripper_width_target,
                            'velocity': 10.0,
                            'force_limit': grasp_force
                        })
                    self.last_gripper_width_target[1] = right_gripper_width_target

            self.gripper_interval_count += 1
            if self.gripper_interval_count % gripper_control_time_interval == 0:
                self.gripper_interval_count = 0

            # Get the target position from Unity and transform it to the robot base frame
            r_pos_from_unity = self.transforms.unity2robot_frame(np.array(mes.rightHand.pos + mes.rightHand.quat), False)
            l_pos_from_unity = self.transforms.unity2robot_frame(np.array(mes.leftHand.pos + mes.leftHand.quat), True)

            if self.left_homing_state:
                logger.debug("left still in homing state")
                self.left_tracking_state = False
            else:
                if mes.leftHand.cmd == 2:
                    if self.left_tracking_state:
                        logger.info("left robot stop tracking")
                        self.left_tracking_state = False
                        self.send_command('/stop_gripper/left', {})
                    else:
                        logger.info("left robot start tracking")
                        self.set_start_tcp(left_tcp, l_pos_from_unity, is_left=True)

            if self.right_homing_state:
                logger.debug("right still in homing state")
                self.right_tracking_state = False
            else:
                if mes.rightHand.cmd == 2:
                    if self.right_tracking_state:
                        logger.info("right robot stop tracking")
                        self.right_tracking_state = False
                        self.send_command('/stop_gripper/right', {})
                    else:
                        logger.info("right robot start tracking")
                        self.set_start_tcp(right_tcp, r_pos_from_unity, is_left=False)

            if not self.left_homing_state and not self.right_homing_state:
                threshold = 0.3
                if self.left_tracking_state:
                    left_target = self.calc_relative_target(l_pos_from_unity, is_left=True)
                    if np.linalg.norm(left_target[:3] - left_tcp[:3]) > threshold:
                        if self.left_tracking_state:
                            logger.info("left robot lost sync")
                        self.left_tracking_state = False
                    else:
                        self.send_command('/move_tcp/left', {'target_tcp': left_target.tolist()})

                if self.right_tracking_state:
                    right_target = self.calc_relative_target(r_pos_from_unity, is_left=False)
                    if np.linalg.norm(right_target[:3] - right_tcp[:3]) > threshold:
                        if self.right_tracking_state:
                            logger.info("right robot lost sync")
                        self.right_tracking_state = False
                    else:
                        self.send_command('/move_tcp/right', {'target_tcp': right_target.tolist()})

            # logger.debug(f"Received data from VR: {mes}")
            end_time = time.time()
            precise_sleep(self.control_cycle_time - (end_time - start_time))
