import threading
import multiprocessing
import time

import rclpy
from real_world.real_world_transforms import RealWorldTransforms
from real_world.teleoperation.teleop_server import TeleopServer
from real_world.publisher.bimanual_robot_publisher import BimanualRobotPublisher
from real_world.robot.bimanual_flexiv_server import BimanualFlexivServer
import hydra
from omegaconf import DictConfig
from loguru import logger

def create_robot_publisher_node(cfg: DictConfig, transforms: RealWorldTransforms):
    rclpy.init(args=None)
    robot_publisher_node = BimanualRobotPublisher(transforms=transforms,
                                                  **cfg.task.publisher.robot_publisher)
    try:
        rclpy.spin(robot_publisher_node)
    except KeyboardInterrupt:
        robot_publisher_node.destroy_node()
        # rclpy.shutdown()

@hydra.main(
    config_path="config", config_name="default", version_base="1.3"
)
def main(cfg: DictConfig):
    # create robot server
    robot_server = BimanualFlexivServer(**cfg.task.robot_server)
    robot_server_thread = threading.Thread(target=robot_server.run, daemon=True)
    # start the robot server
    robot_server_thread.start()
    # wait for the robot server to start
    time.sleep(1)

    # create teleop server
    transforms = RealWorldTransforms(option=cfg.task.transforms)
    teleop_server = TeleopServer(robot_server_ip=cfg.task.robot_server.host_ip,
                                 robot_server_port=cfg.task.robot_server.port,
                                 transforms=transforms,
                                 **cfg.task.teleop_server)
    teleop_process = multiprocessing.Process(target=teleop_server.run)

    publisher_process = multiprocessing.Process(target=create_robot_publisher_node, args=(cfg, transforms))
    try:
        publisher_process.start()
        teleop_process.start()

        publisher_process.join()
        robot_server_thread.join()
    except KeyboardInterrupt:
        teleop_process.terminate()
        publisher_process.terminate()
    finally:
        # Wait for the process and thread to finish
        teleop_process.join()
        logger.info("Teleop server process finished")
        publisher_process.join()
        logger.info("Publisher process finished")


if __name__ == "__main__":
    main()