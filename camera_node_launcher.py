'''
This file reads mapping from the api server
and launches all the nodes
'''

import multiprocessing
import threading
import time
import os
import signal

import requests
import hydra
from omegaconf import DictConfig, OmegaConf
import rclpy
from loguru import logger

from real_world.publisher.realsense_camera_publisher import RealsenseCameraPublisher
from real_world.device_mapping.device_mapping_server import DeviceToTopic, DeviceMappingServer

# add this to prevent assigning too may threads when using numpy
os.environ["OPENBLAS_NUM_THREADS"] = "12"
os.environ["MKL_NUM_THREADS"] = "12"
os.environ["NUMEXPR_NUM_THREADS"] = "12"
os.environ["OMP_NUM_THREADS"] = "12"

import cv2
# add this to prevent assigning too may threads when using open-cv
cv2.setNumThreads(12)

class CameraWorker:
    def __init__(self, camera_config):
        self.camera_config = camera_config
        if camera_config.camera_type == 'D400':
            self.camera_publisher = RealsenseCameraPublisher(**camera_config)
        elif camera_config.camera_type == 'USB':
            self.camera_publisher = UsbCameraPublisher(**camera_config)
        else:
            raise NotImplementedError
    def handle_signal(self, signum, frame):
        self.camera_publisher.stop()
        logger.info(f"Stopped {self.camera_config.camera_name} camera publisher")
        self.camera_publisher.destroy_node()
        # rclpy.shutdown()

def start_camera_publisher(camera_config):
    rclpy.init(args=None)
    worker = CameraWorker(camera_config)
    signal.signal(signal.SIGUSR1, worker.handle_signal)

    logger.info(f"Starting {camera_config.camera_name} camera publisher")
    rclpy.spin(worker.camera_publisher)

@hydra.main(
    config_path="config", config_name="default", version_base="1.3"
)
def main(cfg: DictConfig):
    try:
        device_mapper_server = DeviceMappingServer(publisher_cfg=cfg.task.publisher,
                                                   **cfg.task.device_mapping_server)
        device_mapping_thread = threading.Thread(target=device_mapper_server.run, daemon=True)
        device_mapping_thread.start()
        time.sleep(1)

        # require the latest mapping of name and topic from fastAPI
        response = requests.get(f"http://{cfg.task.device_mapping_server.host_ip}:{cfg.task.device_mapping_server.port}/get_mapping")
        device_to_topic = DeviceToTopic.model_validate(response.json())
        logger.info(f"Find {len(device_to_topic.realsense)} realsense cameras")

        # launch the subprocesses based on the mapping from fastapi server
        processes = []
        # Handle realsense cameras
        for camera_name, camera_info in device_to_topic.realsense.items():
            camera_config = None
            for cam in cfg.task.publisher.realsense_camera_publisher:
                if cam.camera_name == camera_name:
                    camera_config = cam
                    break
            if camera_config:
                # Assigning device_id and type
                OmegaConf.set_struct(camera_config, False)
                camera_config.camera_serial_number = camera_info.device_id
                OmegaConf.set_struct(camera_config, True)

                p = multiprocessing.Process(target=start_camera_publisher, args=(camera_config,))
                processes.append(p)
                p.start()

        device_mapping_thread.join()
    except KeyboardInterrupt:
        for p in processes:
            os.kill(p.pid, signal.SIGUSR1)
        time.sleep(2)
    finally:
        for p in processes:
            p.terminate()
        for p in processes:
            p.join()
        logger.info("All Camera publishers shutdown")

if __name__ == "__main__":
    main()