'''
This file initiate the DeviceMappingServer
The server then dynamic maintain the mapping between
cameras and the topics
'''

from fastapi import FastAPI
import uvicorn
from omegaconf import DictConfig
import subprocess
import pyrealsense2 as rs
from pydantic import BaseModel
from typing import Dict, Optional
from loguru import logger

class RealsenseCameraInfo(BaseModel):
    topic_image: str
    topic_pointcloud: str = None
    device_id: str
    type: str

class DeviceToTopic(BaseModel):
    realsense: Dict[str, RealsenseCameraInfo] = {}

class DeviceMappingServer:
    """Server class that defines the device mapping (device to ROS topic name)"""
    def __init__(self, publisher_cfg: DictConfig, host_ip: str = '127.0.0.1', port: int = 8062):
        self.host_ip = host_ip
        self.port = port

        self.app = FastAPI()
        self.device_to_topic_mapping = DeviceToTopic()
        self.init_mapping(publisher_cfg)
        self.setup_routs()

    def setup_routs(self):
        @self.app.get("/get_mapping", response_model=DeviceToTopic)
        def get_mapping() -> DeviceToTopic:
            return self.device_to_topic_mapping

    def init_mapping(self, publisher_cfg: DictConfig):
        '''
        get the device ids of the cameras in sequence
        '''

        # realsence camera
        for rs_cam in publisher_cfg.realsense_camera_publisher:
            context = rs.context()
            for device in context.query_devices():
                if device.get_info(rs.camera_info.serial_number) == rs_cam.camera_serial_number:
                    self.device_to_topic_mapping.realsense[rs_cam.camera_name] = RealsenseCameraInfo(
                        topic_image=f"{rs_cam.camera_name}/color/image_raw",
                        topic_pointcloud=f'/{rs_cam.camera_name}/depth/points' if rs_cam.enable_pcd_publisher else '',
                        device_id=rs_cam.camera_serial_number,
                        type="realsense"
                    )
                    break

    def run(self):
        logger.info(f"Device mapping server is running on {self.host_ip}:{self.port}")
        uvicorn.run(self.app, host=self.host_ip, port=self.port)
