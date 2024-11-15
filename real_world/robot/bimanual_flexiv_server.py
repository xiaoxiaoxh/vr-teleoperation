import threading
from typing import List, Dict
import time
import uvicorn
from fastapi import FastAPI, HTTPException
from loguru import logger

from real_world.robot.single_flexiv_controller import FlexivController
from common.data_models import BimanualRobotStates, MoveGripperRequest, TargetTCPRequest

class BimanualFlexivServer():
    """
    Bimanual Flexiv Server Class
    """
    # TODO: use UDP to respond
    def __init__(self,
                 host_ip="192.168.2.187",
                 port: int = 8092,
                 left_robot_ip="192.168.2.110",
                 right_robot_ip="192.168.2.111",
                 use_planner: bool = False
                 ) -> None:
        self.host_ip = host_ip
        self.port = port
        self.left_robot = FlexivController(local_ip=host_ip,
                                           robot_ip=left_robot_ip, )
        self.right_robot = FlexivController(local_ip=host_ip,
                                            robot_ip=right_robot_ip, )

        self.left_robot.robot.setMode(self.left_robot.mode.NRT_CARTESIAN_MOTION_FORCE)
        self.right_robot.robot.setMode(self.right_robot.mode.NRT_CARTESIAN_MOTION_FORCE)

        # open the gripper
        self.left_robot.gripper.move(0.1, 10, 0)
        self.right_robot.gripper.move(0.1, 10, 0)

        if use_planner:
            # TODO: support bimanual planner
            raise NotImplementedError
        else:
            self.planner = None

        self.app = FastAPI()
        # Start the receiving command thread
        self.setup_routes()

    def setup_routes(self):
        @self.app.post('/clear_fault')
        async def clear_fault() -> List[str]:
            if self.left_robot.robot.isFault():
                logger.warning("Fault occurred on left robot server, trying to clear ...")
                thread_left = threading.Thread(target=self.left_robot.clear_fault)
                thread_left.start()
            else:
                thread_left = None
            if self.right_robot.robot.isFault():
                logger.warning("Fault occurred on right robot server, trying to clear ...")
                thread_right = threading.Thread(target=self.right_robot.clear_fault)
                thread_right.start()
            else:
                thread_right = None
            # Wait for both threads to finish
            fault_msgs = []
            if thread_left is not None:
                thread_left.join()
                fault_msgs.append("Left robot fault cleared")
            if thread_right is not None:
                thread_right.join()
                fault_msgs.append("Right robot fault cleared")
            return fault_msgs

        @self.app.get('/get_current_robot_states')
        async def get_current_robot_states() -> BimanualRobotStates:
            return BimanualRobotStates(leftRobotTCP=self.left_robot.get_current_tcp(),
                                       rightRobotTCP=self.right_robot.get_current_tcp(),
                                       leftGripperState=[self.left_robot.get_current_gripper_width(),
                                                         self.left_robot.get_current_gripper_force()],
                                       rightGripperState=[self.right_robot.get_current_gripper_width(),
                                                        self.right_robot.get_current_gripper_force()])

        @self.app.post('/move_gripper/{robot_side}')
        async def move_gripper(robot_side: str, request: MoveGripperRequest) -> Dict[str, str]:
            if robot_side not in ['left', 'right']:
                raise HTTPException(status_code=400, detail="Invalid robot side. Use 'left' or 'right'.")

            robot_gripper = self.left_robot.gripper if robot_side == 'left' else self.right_robot.gripper
            robot_gripper.move(request.width, request.velocity, request.force_limit)
            return {
                "message": f"{robot_side.capitalize()} gripper moving to width {request.width} "
                           f"with velocity {request.velocity} and force limit {request.force_limit}"}

        @self.app.post('/move_gripper_force/{robot_side}')
        async def move_gripper_force(robot_side: str, request: MoveGripperRequest) -> Dict[str, str]:
            if robot_side not in ['left', 'right']:
                raise HTTPException(status_code=400, detail="Invalid robot side. Use 'left' or 'right'.")

            robot_gripper = self.left_robot.gripper if robot_side == 'left' else self.right_robot.gripper
            # use force control mode to grasp
            robot_gripper.grasp(request.force_limit)
            return {
                "message": f"{robot_side.capitalize()} gripper grasp with force limit {request.force_limit}"}

        @self.app.post('/stop_gripper/{robot_side}')
        async def stop_gripper(robot_side: str) -> Dict[str, str]:
            if robot_side not in ['left', 'right']:
                raise HTTPException(status_code=400, detail="Invalid robot side. Use 'left' or 'right'.")

            robot_gripper = self.left_robot.gripper if robot_side == 'left' else self.right_robot.gripper

            robot_gripper.stop()
            return {"message": f"{robot_side.capitalize()} gripper stopping"}

        @self.app.post('/move_tcp/{robot_side}')
        async def move_tcp(robot_side: str, request: TargetTCPRequest) -> Dict[str, str]:
            if robot_side not in ['left', 'right']:
                raise HTTPException(status_code=400, detail="Invalid robot side. Use 'left' or 'right'.")

            robot = self.left_robot if robot_side == 'left' else self.right_robot

            robot.tcp_move(request.target_tcp)
            # logger.debug(f"{robot_side.capitalize()} robot moving to target tcp {request.target_tcp}")
            return {"message": f"{robot_side.capitalize()} robot moving to target tcp {request.target_tcp}"}

        @self.app.get('/get_current_tcp/{robot_side}')
        async def get_current_tcp(robot_side: str) -> List[float]:
            if robot_side not in ['left', 'right']:
                raise HTTPException(status_code=400, detail="Invalid robot side. Use 'left' or 'right'.")

            robot = self.left_robot if robot_side == 'left' else self.right_robot

            return robot.get_current_tcp()

        @self.app.post('/birobot_go_home')
        async def birobot_go_home() -> Dict[str, str]:
            if self.planner is None:
                return {"message": "Planner is not available"}
            self.left_robot.robot.setMode(self.left_robot.mode.NRT_JOINT_POSITION)
            self.right_robot.robot.setMode(self.right_robot.mode.NRT_JOINT_POSITION)

            current_q = self.left_robot.get_current_q() + self.right_robot.get_current_q()
            waypoints = self.planner.getGoHomeTraj(current_q)

            for js in waypoints:
                print(js)
                self.left_robot.move(js[:7])
                self.right_robot.move(js[7:])
                time.sleep(0.01)

            self.left_robot.robot.setMode(self.left_robot.mode.NRT_CARTESIAN_MOTION_FORCE)
            self.right_robot.robot.setMode(self.right_robot.mode.NRT_CARTESIAN_MOTION_FORCE)
            return {"message": "Bimanual robots have gone home"}

    def run(self):
        logger.info(f"Start Bimanual Robot Fast-API Server at {self.host_ip}:{self.port}")
        uvicorn.run(self.app, host=self.host_ip, port=self.port, log_level="critical")

def main():
    from hydra import initialize, compose
    from hydra.utils import instantiate

    with initialize(config_path='../../../config', version_base="1.3"):
        # config is relative to a module
        cfg = compose(config_name="default")

    robot_server = instantiate(cfg.robot_server)
    robot_server.run()


if __name__ == "__main__":
    main()