import threading
import time

import rclpy
from real_world.real_world_transforms import RealWorldTransforms
import requests
from real_world.robot.bimanual_flexiv_server import BimanualFlexivServer
from real_world.publisher.bimanual_robot_publisher import BimanualRobotPublisher
from hydra import initialize, compose
from hydra.utils import instantiate
from loguru import logger


def send_command(cfg, endpoint: str, data: dict = None):
    url = f"http://{cfg.robot_server.host_ip}:{cfg.robot_server.port}{endpoint}"
    if 'get' in endpoint:
        response = requests.get(url)
    else:
        logger.debug(f"send command {endpoint} with {data}")
        response = requests.post(url, json=data)
        logger.debug(f"send command {endpoint}, get response {response.json()}")
    response.raise_for_status()  # Raise an error for bad responses
    return response.json()

def test_gripper(cfg):
    send_command(cfg, "/move_gripper/right", {
        'width': 0.02,
        'velocity': 10,
        'force_limit': 10
    })

def test_clear_fault(cfg):
    send_command(cfg, "/clear_fault")

def main_with_robot_server_init():
    with initialize(config_path='../config/task', version_base="1.3"):
        # config is relative to a module
        cfg = compose(config_name="bimanual_two_realsense_left_10fps")

    # create robot server
    robot_server: BimanualFlexivServer = instantiate(cfg.robot_server)
    robot_server_thread = threading.Thread(target=robot_server.run, daemon=True)

    transforms = RealWorldTransforms(option=cfg.transforms)

    # TODO: support args for rclpy
    rclpy.init(args=None)
    robot_publisher_node = BimanualRobotPublisher(robot_server_ip=cfg.robot_server.host_ip,
                                                  robot_server_port=cfg.robot_server.port,
                                                  transforms=transforms,
                                                  **cfg.publisher.robot_publisher)
    publisher_thread = threading.Thread(target=rclpy.spin, args=(robot_publisher_node,), daemon=True)
    try:
        robot_server_thread.start()
        time.sleep(2)
        publisher_thread.start()

        time.sleep(2)
        # tests
        test_clear_fault(cfg)
        test_gripper(cfg)

        robot_server_thread.join()
    except KeyboardInterrupt:
        logger.debug("KeyboardInterrupt")
    finally:
        # Wait for the process and thread to finish
        robot_publisher_node.destroy_node()
        rclpy.shutdown()
        logger.info("Publisher node shutdown")

def main_without_robot_server():
    with initialize(config_path='../config', version_base="1.3"):
        # config is relative to a module
        cfg = compose(config_name="default")

    test_clear_fault(cfg)
    test_gripper(cfg)

if __name__ == "__main__":
    main_with_robot_server_init()
    # main_without_robot_server()