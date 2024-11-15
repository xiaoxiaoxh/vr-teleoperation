# utils.py
# dynamically get the currently running topics
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from real_world.device_mapping.device_mapping_server import DeviceToTopic
from loguru import logger

def get_topic_and_type(device_to_topic: DeviceToTopic):
    subs_name_type = []

    for camera_name, info in device_to_topic.realsense.items():
        logger.debug(f'camera info: {info}')
        subs_name_type.append((f'/{camera_name}/color/image_raw', Image))
        if info.topic_pointcloud != '':
            subs_name_type.append((f'/{camera_name}/depth/points', PointCloud2))

    subs_name_type.extend([
        ('/tcp_pose_left', PoseStamped),
        ('/tcp_pose_right', PoseStamped),
        ('/left_gripper_state', JointState),
        ('/right_gripper_state', JointState)
    ])

    return subs_name_type


