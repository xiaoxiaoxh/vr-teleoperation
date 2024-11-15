from hydra import initialize, compose
import rclpy
import os.path as osp
import sys
from loguru import logger
from real_world.real_world_transforms import RealWorldTransforms
from real_world.teleoperation.data_recorder import DataRecorder

import os
# add this to prevent assigning too may threads when using numpy
os.environ["OPENBLAS_NUM_THREADS"] = "12"
os.environ["MKL_NUM_THREADS"] = "12"
os.environ["NUMEXPR_NUM_THREADS"] = "12"
os.environ["OMP_NUM_THREADS"] = "12"

import cv2
# add this to prevent assigning too may threads when using open-cv
cv2.setNumThreads(12)

def main(args=None):
    import argparse
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Data Recorder')
    parser.add_argument('--save_base_dir', type=str, default='/home/xuehan/Desktop/record_data', help='Base directory to save the data')
    parser.add_argument('--save_file_dir', type=str, default='test')
    parser.add_argument('--save_file_name', type=str, default='test.pkl', help='File name of the save file')
    parser.add_argument('--save_to_disk', action='store_true', default=False, help='Whether to save the data to disk')
    parser.add_argument('--debug', action='store_true', default=False, help='Whether to print debug messages')
    args = parser.parse_args()
        
    with initialize(config_path='config', version_base="1.1"):
        cfg = compose(config_name="default")

    rclpy_args = sys.argv
    rclpy.init(args=rclpy_args)
    

    base_dir = osp.join(args.save_base_dir, args.save_file_dir)
    save_path = osp.join(base_dir, args.save_file_name)
    
    transforms = RealWorldTransforms(option=cfg.task.transforms)
    node = DataRecorder(transforms,
                        save_path=save_path,
                        debug=args.debug,
                        device_mapping_server_ip=cfg.task.device_mapping_server.host_ip,
                        device_mapping_server_port=cfg.task.device_mapping_server.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if args.save_to_disk:
            node.save()
        else:
            logger.info("Data not saved to disk, quitting program now...")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()