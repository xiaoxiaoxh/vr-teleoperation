import pickle
import os
from loguru import logger
import zarr
import numpy as np
import os.path as osp
import py_cli_interaction
import tqdm
from hydra import initialize, compose
from real_world.real_world_transforms import RealWorldTransforms
from common.visualization_utils import visualize_pcd_from_numpy, visualize_rgb_image
from real_world.post_process_utils import DataPostProcessingManager

import argparse

DEBUG = False
USE_ABSOLUTE_ACTION = True
USE_BINARY_GRIPPER_ACTION = False
SENSOR_MODE = 'single_arm_two_realsense'
USE_MERGED_PCD = True

def convert_gripper_action(gripper_width_arrays: np.ndarray):
    EPS = 1e-3
    max_gripper_width = np.max(gripper_width_arrays)
    min_gripper_width = np.min(gripper_width_arrays)
    # convert gripper width to binary action (min/max width)
    gripper_action_arrays = np.where((gripper_width_arrays[:-1] > gripper_width_arrays[1:]) |
                                     (np.abs(gripper_width_arrays[:-1] - min_gripper_width) < EPS), min_gripper_width,
                                     np.where((gripper_width_arrays[:-1] < gripper_width_arrays[1:]) |
                                     (np.abs(gripper_width_arrays[:-1] - max_gripper_width) < EPS), max_gripper_width,
                                      np.where(np.abs(gripper_width_arrays[:-1] - max_gripper_width) < EPS, max_gripper_width, min_gripper_width)))
    gripper_action_arrays = np.concatenate([gripper_action_arrays, gripper_action_arrays[:, np.newaxis][-1]], axis=0)
    return gripper_action_arrays

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag", type=str, default="test")
    args = parser.parse_args()

    tag = args.tag
    data_dir = f'/root/record_data/{tag}'
    save_data_dir = f'/root/record_data/{tag}_zarr'
    save_data_path = osp.join(osp.join(osp.abspath(os.getcwd()), save_data_dir, f'{tag}.zarr'))
    os.makedirs(save_data_dir, exist_ok=True)
    if os.path.exists(save_data_path):
        logger.info('Data already exists at {}'.format(save_data_path))
        # use py_cli_interaction to ask user if they want to overwrite the data
        if py_cli_interaction.parse_cli_bool('Do you want to overwrite the data?', default_value=True):
            logger.warning('Overwriting {}'.format(save_data_path))
            os.system('rm -rf {}'.format(save_data_path))

    # loading config for transforms
    with initialize(config_path='config', version_base="1.3"):
        # config is relative to a module
        cfg = compose(config_name="default")
    transforms = RealWorldTransforms(option=cfg.task.transforms)

    data_processing_manager = DataPostProcessingManager(transforms=transforms,
                                                        mode=SENSOR_MODE,
                                                        use_6d_rotation=True,
                                                        use_merged_pcd=USE_MERGED_PCD)

    # storage
    external_img_arrays = []
    left_wrist_img_arrays = []
    right_wrist_img_arrays = []
    external_pcd_arrays = []
    left_wrist_pcd_arrays = []
    right_wrist_pcd_arrays = []
    merged_pcd_arrays = []
    state_arrays = []
    action_arrays = []
    episode_ends_arrays = []
    total_count = 0
    # find all the files in the data directory
    data_files = sorted([f for f in os.listdir(data_dir) if f.endswith('.pkl')])
    for seq_idx, data_file in tqdm.tqdm(enumerate(data_files)):
        data_path = osp.join(data_dir, data_file)

        abs_path = os.path.abspath(data_path)
        save_data_path = os.path.abspath(save_data_path)
        logger.info(f'Loading data from {abs_path}')

        # Load the data
        with open(data_path, 'rb') as f:
            data = pickle.load(f)

        for step_idx, sensor_msg in enumerate(data.sensorMessages):
            total_count += 1
            logger.info(f'Processing {step_idx}th sensor message in sequence {seq_idx}')

            obs_dict = data_processing_manager.convert_sensor_msg_to_obs_dict(sensor_msg)

            state_arrays.append(obs_dict['state'])
            action_arrays.append(obs_dict['action'])

            if USE_MERGED_PCD:
                merged_pcd_arrays.append(obs_dict['merged_pcd'])
            external_img_arrays.append(obs_dict['external_img'])
            external_pcd_arrays.append(obs_dict['external_pcd'])
            left_wrist_img_arrays.append(obs_dict['left_wrist_img'])
            left_wrist_pcd_arrays.append(obs_dict['left_wrist_pcd'])

            if DEBUG:
                logger.debug(f'leftRobotState {sensor_msg.leftRobotState}')
                logger.debug(f'rightRobotState {sensor_msg.rightRobotState}')
                logger.debug(f'leftRobotAction {sensor_msg.leftRobotAction}')
                logger.debug(f'rightRobotAction {sensor_msg.rightRobotAction}')

                # visualize external camera image
                visualize_rgb_image(sensor_msg.externalCameraRGB, 'External Camera RGB')
                # visualize left wrist camera image
                visualize_rgb_image(sensor_msg.leftWristCameraRGB, 'Left Wrist Camera RGB')
                # visualize right wrist camera image
                visualize_rgb_image(sensor_msg.rightWristCameraRGB, 'Right Wrist Camera RGB')

                # visualize raw external_pcd
                visualize_pcd_from_numpy(sensor_msg.externalCameraPointCloud.astype(np.float32))
                # visualize processed external_pcd
                visualize_pcd_from_numpy(obs_dict['external_pcd'].astype(np.float32))

                # visualize raw left_wrist_pcd
                visualize_pcd_from_numpy(sensor_msg.leftWristCameraPointCloud.astype(np.float32))
                # visualize processed left_wrist_pcd
                visualize_pcd_from_numpy(obs_dict['left_wrist_pcd'].astype(np.float32))

                if USE_MERGED_PCD:
                    # visualize merged point cloud
                    visualize_pcd_from_numpy(obs_dict['merged_pcd'].astype(np.float32))

            del sensor_msg, obs_dict

        episode_ends_arrays.append(total_count)

    # create zarr file
    zarr_root = zarr.group(save_data_path)
    zarr_data = zarr_root.create_group('data')
    zarr_meta = zarr_root.create_group('meta')

    external_img_arrays = np.stack(external_img_arrays, axis=0)
    left_wrist_img_arrays = np.stack(left_wrist_img_arrays, axis=0)

    action_arrays = np.stack(action_arrays, axis=0)
    state_arrays = np.stack(state_arrays, axis=0)
    episode_ends_arrays = np.array(episode_ends_arrays)
    if USE_MERGED_PCD:
        merged_pcd_arrays = np.stack(merged_pcd_arrays, axis=0)
    else:
        external_pcd_arrays = np.stack(external_pcd_arrays, axis=0)
        left_wrist_pcd_arrays = np.stack(left_wrist_pcd_arrays, axis=0)

    if USE_ABSOLUTE_ACTION:
        # override action to absolute value
        num_samples = state_arrays.shape[0]
        new_action_arrays = state_arrays[1:, ...].copy()
        action_arrays = np.concatenate([new_action_arrays, new_action_arrays[-1][np.newaxis, :]], axis=0)
        if USE_BINARY_GRIPPER_ACTION:
            left_gripper_width_arrays = state_arrays[:, 9]
            left_gripper_action_arrays = convert_gripper_action(left_gripper_width_arrays)
            action_arrays[:, 9] = left_gripper_action_arrays

            right_gripper_width_arrays = state_arrays[:, 20]
            right_gripper_action_arrays = convert_gripper_action(right_gripper_width_arrays)
            action_arrays[:, 20] = right_gripper_action_arrays

    # create
    compressor = zarr.Blosc(cname='zstd', clevel=3, shuffle=1)
    external_img_chunk_size = (100, external_img_arrays.shape[1], external_img_arrays.shape[2], external_img_arrays.shape[3])
    wrist_img_chunk_size = (100, left_wrist_img_arrays.shape[1], left_wrist_img_arrays.shape[2], left_wrist_img_arrays.shape[3])
    if USE_MERGED_PCD:
        point_cloud_chunk_size = (100, merged_pcd_arrays.shape[1], merged_pcd_arrays.shape[2])
    else:
        point_cloud_chunk_size = (100, external_pcd_arrays.shape[1], external_pcd_arrays.shape[2])
    if len(action_arrays.shape) == 2:
        action_chunk_size = (100, action_arrays.shape[1])
    elif len(action_arrays.shape) == 3:
        action_chunk_size = (100, action_arrays.shape[1], action_arrays.shape[2])
    else:
        raise NotImplementedError

    zarr_data.create_dataset('action', data=action_arrays, chunks=action_chunk_size, dtype='float32', overwrite=True,
                             compressor=compressor)
    zarr_data.create_dataset('state', data=state_arrays, chunks=(100, state_arrays.shape[1]), dtype='float32',
                             overwrite=True, compressor=compressor)
    zarr_meta.create_dataset('episode_ends', data=episode_ends_arrays, chunks=(100,), dtype='int64', overwrite=True,
                             compressor=compressor)
    zarr_data.create_dataset('external_img', data=external_img_arrays, chunks=external_img_chunk_size, dtype='uint8', overwrite=True,
                             compressor=compressor)
    zarr_data.create_dataset('left_wrist_img', data=left_wrist_img_arrays, chunks=wrist_img_chunk_size, dtype='uint8')
    if USE_MERGED_PCD:
        zarr_data.create_dataset('merged_point_cloud', data=merged_pcd_arrays, chunks=point_cloud_chunk_size, dtype='float16',
                                 overwrite=True, compressor=compressor)
    else:
        zarr_data.create_dataset('external_point_cloud', data=external_pcd_arrays, chunks=point_cloud_chunk_size, dtype='float16',
                                 overwrite=True, compressor=compressor)
        zarr_data.create_dataset('left_wrist_point_cloud', data=left_wrist_pcd_arrays, chunks=point_cloud_chunk_size, dtype='float16',
                                    overwrite=True, compressor=compressor)

    # print zarr data structure
    logger.info('Zarr data structure')
    logger.info(zarr_data.tree())
    logger.info(f'Total count: {total_count}')
    logger.info(f'Save data at {save_data_path}')