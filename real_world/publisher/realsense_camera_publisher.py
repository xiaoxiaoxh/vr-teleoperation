import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import copy
import cv2
import os
import matplotlib.pyplot as plt
from loguru import logger
from common.time_utils import convert_float_to_ros_time
from ros2_numpy.point_cloud2 import array_to_pointcloud2
from common.pcd_utils import random_sample_pcd
import time
import socket
import math
import bson

class RealsenseCameraPublisher(Node):
    """
    Realsense Camera publisher class
    """
    def __init__(self,
                 camera_serial_number: str = '036422060422',
                 camera_type: str = 'D400',  # L500
                 camera_name: str = 'camera_base',
                 rgb_resolution: tuple = (640, 480),
                 depth_resolution: tuple = (640, 480),
                 fps: int = 30,
                 decimate: int = 2,  # (0-4) decimation_filter magnitude for point cloud
                 enable_pcd_publisher: bool = True,
                 time_check = False,
                 random_sample_point_num: int = 10000,
                 enable_streaming: bool = False,
                 streaming_server_ip: str = '127.0.0.1',
                 streaming_server_port: int = 10004,
                 streaming_quality: int = 10,
                 streaming_chunk_size: int = 1024,
                 streaming_display_params_list: list = None,
                 debug: bool = False
                 ):
        node_name = f'{camera_name}_publisher'
        super().__init__(node_name)
        self.camera_serial_number = camera_serial_number
        self.camera_type = camera_type
        self.camera_name = camera_name
        self.fps = fps
        self.rgb_resolution = rgb_resolution
        self.depth_resolution = depth_resolution
        self.time_check = time_check
        self.random_sample_point_num = random_sample_point_num
        self.enable_pcd_publisher = enable_pcd_publisher

        # streaming configuration
        self.enable_streaming = enable_streaming
        if self.enable_streaming:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.streaming_server_ip = streaming_server_ip
            self.streaming_server_port = streaming_server_port
            self.streaming_quality = streaming_quality
            self.streaming_chunk_size = streaming_chunk_size
            streaming_display_params_list = [{k:list(v) for k, v in d.items()} for d in streaming_display_params_list]
            self.streaming_display_params_list = streaming_display_params_list

        self.debug = debug

        self.color_publisher_ = self.create_publisher(Image, f'/{camera_name}/color/image_raw', 10)
        if self.enable_pcd_publisher:
            self.depth_publisher_ = self.create_publisher(PointCloud2, f'/{camera_name}/depth/points', 10)
        self.timer = self.create_timer(1 / fps, self.timer_callback)
        self.pipeline = None
        self.timestamp_offset = None
        self.depth_scale = None

        self.prev_time = time.time()
        self.last_print_time = time.time()  # Add a variable to keep track of the last print time
        self.frame_count = 0
        self.fps_list = []
        self.frame_intervals = []
        self.last_frame_time = None

        # Create a decimation filter
        self.decimate_filter = rs.decimation_filter()
        self.decimate_filter.set_option(rs.option.filter_magnitude, 2 ** decimate)

        # Start the camera
        self.start()

    def set_exposure(self, exposure=None, gain=None):
        """
        exposure: (1, 10000) 100us unit. (0.1 ms, 1/10000s)
        gain: (0, 128)
        """

        if exposure is None and gain is None:
            # auto exposure
            self.color_sensor.set_option(rs.option.enable_auto_exposure, 1.0)
        else:
            # manual exposure
            self.color_sensor.set_option(rs.option.enable_auto_exposure, 0.0)
            if exposure is not None:
                self.color_sensor.set_option(rs.option.exposure, exposure)
            if gain is not None:
                self.color_sensor.set_option(rs.option.gain, gain)

    def set_white_balance(self, white_balance=None):
        if white_balance is None:
            self.color_sensor.set_option(rs.option.enable_auto_white_balance, 1.0)
        else:
            self.color_sensor.set_option(rs.option.enable_auto_white_balance, 0.0)
            self.color_sensor.set_option(rs.option.white_balance, white_balance)

    def start(self):
        # get the context of the connected devices
        context = rs.context()
        devices = context.query_devices()

        # check if there are connected devices
        if len(devices) == 0:
            logger.error("No connected devices found")
            raise Exception("No connected devices found")

        config = rs.config()
        is_camera_valid = False
        for device in devices:
            # check if the device serial number matches the provided serial number
            serial_number = device.get_info(rs.camera_info.serial_number)
            if serial_number == self.camera_serial_number:
                is_camera_valid = True
                break

        # if the provided camera is not found, raise an exception
        if not is_camera_valid:
            logger.error("Camera with serial number {} not found".format(self.camera_serial_number))
            raise Exception("Camera with serial number {} not found".format(self.camera_serial_number))

        # Start the camera
        config.enable_device(self.camera_serial_number)
        self.pipeline = rs.pipeline()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        assert device_product_line == self.camera_type, f'With {self.camera_name}, Camera type does not match the camera product line.'
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_sensor = device.first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # report global time
        # https://github.com/IntelRealSense/librealsense/pull/3909
        self.color_sensor = device.first_color_sensor()
        self.color_sensor.set_option(rs.option.global_time_enabled, 1)
        # realsense exposure
        self.set_exposure(exposure=120, gain=0)
        # realsense white balance
        self.set_white_balance(white_balance=5900)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # set the resolution and format of the camera
        config.enable_stream(rs.stream.depth, self.depth_resolution[0], self.depth_resolution[1], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.rgb_resolution[0], self.rgb_resolution[1], rs.format.bgr8, 30)
        self.pipeline.start(config)
        logger.debug("Camera started!")

        # capture some frames for the camera to stabilize
        logger.debug("Capturing some frames for the camera to stabilize...")
        for _ in range(self.fps):
            self.pipeline.wait_for_frames()

        # Capture initial frames to get initial timestamps
        frames = self.pipeline.wait_for_frames()

        initial_frame = frames.get_color_frame()
        if not initial_frame:
            logger.error("Failed to get initial frame")
            raise ValueError("Failed to get initial frame")

        # convert the camera timestamp to system timestamp
        initial_camera_timestamp = convert_float_to_ros_time(initial_frame.get_timestamp() / 1000)  # convert to time class in ROS
        # we assume that the internal clock of realsense is synchronized with the system clock
        # TODO: measure accurate latency with QR code
        initial_system_timestamp = initial_camera_timestamp

        # Calculate timestamp offset
        # TODO: measure accurate latency with QR code
        self.timestamp_offset = initial_system_timestamp - initial_camera_timestamp
        logger.debug(f"Timestamp offset: {self.timestamp_offset.nanoseconds / 1e6} ms")
        logger.debug("Camera is ready! Start publishing images...")

    def stop(self):
        # Stop the camera
        self.pipeline.stop()
        logger.info("Camera stopped!")

    def convert_to_system_timestamp(self, camera_timestamp: Time) -> Time:
        """
        Convert camera timestamp to system timestamp
        """
        return camera_timestamp + self.timestamp_offset

    def publish_color_image(self, color_frame: rs.composite_frame, camera_timestamp: Time):
        """
        Publish color image
        """
        color_image = copy.deepcopy(np.asanyarray(color_frame.get_data()))
        success, encoded_image = cv2.imencode('.jpg', color_image)
        
        # Fill the message
        msg = Image()
        msg.header.stamp = self.convert_to_system_timestamp(camera_timestamp).to_msg()
        msg.header.frame_id = "camera_color_frame"
        msg.height, msg.width, _ = color_image.shape
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        if success:
            image_bytes = encoded_image.tobytes()
            msg.data = image_bytes
        else:
            logger.debug('fail to image encoding!')
            msg.data = color_image.tobytes()
        
        # msg = numpy_to_image(color_image, "bgr8")
        self.color_publisher_.publish(msg)
    
    
    def publish_point_cloud(self, color_frame: rs.composite_frame, depth_frame: rs.composite_frame, camera_timestamp: Time):
        """
        Publish point cloud with realsense color frame and depth frame
        """
        # Grab new intrinsics (may be changed by decimation)
        depth_intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()

        color_image = np.asanyarray(color_frame.get_data())
        # Convert BGR to RGB
        color_image = np.asarray(color_image[:, :, ::-1], order="C")

        # Resize the color image to the depth image size
        depth_image = np.asanyarray(depth_frame.get_data())
        resized_color_image = cv2.resize(color_image, dsize=(depth_image.shape[1], depth_image.shape[0]),
                                        interpolation=cv2.INTER_AREA)

        # Create a color pointcloud
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(resized_color_image), 
            o3d.geometry.Image(depth_image), 
            depth_scale=1 / self.depth_scale, 
            depth_trunc=3.0, 
            convert_rgb_to_intensity=False)
        
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, 
            o3d.camera.PinholeCameraIntrinsic(
                depth_intrinsics.width, depth_intrinsics.height, 
                depth_intrinsics.fx, depth_intrinsics.fy, 
                depth_intrinsics.ppx, depth_intrinsics.ppy)
            )

        # random sampling
        if self.random_sample_point_num > 0:
            points, colors = random_sample_pcd(pcd, self.random_sample_point_num, return_pcd=False)
        else:
            # Get the points and colors
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)

        # Combine points and colors into a structured array
        r, g, b = (colors * 255).T.astype(np.uint8)
                
        cloud_arr = np.zeros(points.shape[0], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),
                                                 ('r', np.uint8), ('g', np.uint8), ('b', np.uint8)])
        cloud_arr['x'], cloud_arr['y'], cloud_arr['z'] = points[:, 0], points[:, 1], points[:, 2]
        cloud_arr['r'], cloud_arr['g'], cloud_arr['b'] = r, g, b
        
        # Create the point cloud message
        header = Header()
        header.stamp = self.convert_to_system_timestamp(camera_timestamp).to_msg()
        header.frame_id = "camera_depth_frame"
        
        # Use array_to_pointcloud2 to convert the structured array to PointCloud2 message
        pointcloud_msg = array_to_pointcloud2(cloud_arr, stamp=header.stamp, frame_id=header.frame_id)

        # Publish the point cloud
        self.depth_publisher_.publish(pointcloud_msg)

    def send_streaming_msg(self, color_image):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.streaming_quality]
        ret, color_image_encoded = cv2.imencode('.jpg', color_image, encode_param)
        color_image_bytes = color_image_encoded.tobytes()
        packed_data_dict = {"images": [{**display_params, **{"image": color_image_bytes}} for
                             display_params in self.streaming_display_params_list]}
        packed_data = bson.dumps(packed_data_dict)

        arrow_address = (self.streaming_server_ip, self.streaming_server_port)
        chunk_size = self.streaming_chunk_size

        self.socket.sendto(len(packed_data).to_bytes(length=4, byteorder='little', signed=False), arrow_address)
        if self.debug:
            logger.debug(f"Sending streaming image to VR server with size {len(packed_data)}")

        self.socket.sendto(chunk_size.to_bytes(length=4, byteorder='little', signed=False), arrow_address)
        count = math.ceil(len(packed_data) / chunk_size)
        if self.debug:
            logger.debug(f"Sending streaming image to VR server with {count} chunks of size {chunk_size}")

        for i in range(count):
            start = i * chunk_size
            end = (i + 1) * chunk_size
            if end > len(packed_data):
                end = len(packed_data)
            self.socket.sendto(packed_data[start:end], arrow_address)
        if self.debug:
            logger.debug(f"Sent streaming image to VR server")
    
    def timer_callback(self):
        """
        Publish the color and depth frames
        """

        while True:
            # capture frames
            frames = self.pipeline.wait_for_frames()

            # we only record the raw color frame
            raw_color_frame = frames.get_color_frame()

            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)

            # Get aligned frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            # If depth frame or color frame is not available, continue
            if not color_frame or not depth_frame:
                continue

            # Apply decimation filter
            depth_frame = self.decimate_filter.process(depth_frame)

            # get the internal camera timestamp of the color frame
            camera_timestamp = convert_float_to_ros_time(color_frame.get_timestamp() / 1000)  # convert to time class in ROS

            # publish the color image
            self.publish_color_image(raw_color_frame, camera_timestamp)
            if self.enable_pcd_publisher:
                # publish the point cloud
                self.publish_point_cloud(color_frame, depth_frame, camera_timestamp)

            # send streaming image
            if self.enable_streaming:
                color_image = np.asanyarray(color_frame.get_data())
                self.send_streaming_msg(color_image)

            # calculate fps
            self.frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - self.prev_time
            if elapsed_time >= 1.0:
                frame_rate = self.frame_count / elapsed_time
                self.fps_list.append(frame_rate)
                logger.debug(f"Frame rate: {frame_rate:.2f} FPS")
                self.prev_time = current_time
                self.frame_count = 0

            # calculate interval between frames
            if self.last_frame_time is not None:
                frame_interval = (current_time - self.last_frame_time) * 1000
                self.frame_intervals.append(frame_interval)
            self.last_frame_time = current_time

            # Print info and make plots every 5 seconds
            if current_time - self.last_print_time >= 5:
                logger.info(f"Publishing image from {self.camera_name} at timestamp: {camera_timestamp}")
                self.last_print_time = current_time

                save_path = 'data/latency_check/node_launcher_modifying'
                if self.time_check:
                    # fps plot
                    plt.figure()
                    plt.plot(self.fps_list, label='FPS')
                    plt.xlabel('Time (s)')
                    plt.ylabel('FPS')
                    plt.title(f'{self.camera_name} FPS')
                    plt.legend()
                    plt.grid(True)

                    fps_filename = os.path.join(save_path, f'{self.camera_name}_fps.png')
                    plt.savefig(fps_filename)
                    plt.close()

                    # frame interval plot
                    plt.figure()
                    plt.plot(self.frame_intervals, label='Frame Interval (s)')
                    plt.xlabel('Frame')
                    plt.ylabel('Time Interval (ms)')
                    plt.title(f'{self.camera_name} Frame Intervals')
                    plt.legend()
                    plt.grid(True)

                    interval_filename = os.path.join(save_path, f'{self.camera_name}_frame_intervals.png')
                    plt.savefig(interval_filename)
                    plt.close()

                    self.fps_list.clear()
                    self.frame_intervals.clear()
            break


def main(args=None):
    rclpy.init(args=args)

    from hydra import initialize, compose
    from hydra.utils import instantiate

    with initialize(config_path='../../config/task', version_base="1.3"):
        # config is relative to a module
        cfg = compose(config_name="bimanual_two_realsense_left_10fps")

    node = RealsenseCameraPublisher(**cfg.publisher.realsense_camera_publisher[1], debug=True)
    try:
        rclpy.spin(node)
    except IndentationError as e:
        logger.exception(e)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()