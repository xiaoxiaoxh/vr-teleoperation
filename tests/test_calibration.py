import json
import os
import os.path as osp
import numpy as np
import argparse
import py_cli_interaction
import open3d as o3d

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration_base_path", type=str, default="data/calibration")
    args = parser.parse_args()

    __VERSION_CANDIDATES_DIR_ = args.calibration_base_path
    __VERSION_CANDIDATES__ = list(
        filter(
            lambda x: osp.isdir(osp.join(__VERSION_CANDIDATES_DIR_, x)) and "v" in x,
            os.listdir(__VERSION_CANDIDATES_DIR_))
    )
    __VERSION_CANDIDATES__.sort(key=lambda x: int(x[1:]))
    __VERSION__ = __VERSION_CANDIDATES__[
        py_cli_interaction.must_parse_cli_sel("select calibration version", __VERSION_CANDIDATES__)]

    camera_to_left_robot_base_path = osp.join(__VERSION_CANDIDATES_DIR_, __VERSION__, 'external_camera_to_left_robot_base_transform.json')
    with open(camera_to_left_robot_base_path, "r") as f:
        camera_to_left_robot_base_transform = np.array(json.loads(f.read()))

    camera_to_right_robot_base_path = osp.join(__VERSION_CANDIDATES_DIR_, __VERSION__, 'external_camera_to_right_robot_base_transform.json')
    with open(camera_to_right_robot_base_path, "r") as f:
        camera_to_right_robot_base_transform = np.array(json.loads(f.read()))

    # visualize transform in Open3D
    camera_in_camera = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    left_in_camera = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(
        np.linalg.inv(camera_to_left_robot_base_transform))
    right_in_camera = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(
        np.linalg.inv(camera_to_right_robot_base_transform))
    o3d.visualization.draw_geometries(
        [camera_in_camera, left_in_camera, right_in_camera])  # camera coord system

    world_to_left_robot_path = osp.join(__VERSION_CANDIDATES_DIR_, __VERSION__, 'world_to_left_robot_base_transform.json')
    with open(world_to_left_robot_path, "r") as f:
        world_to_left_robot_transform = np.array(json.loads(f.read()))

    world_to_right_robot_path = osp.join(__VERSION_CANDIDATES_DIR_, __VERSION__, 'world_to_right_robot_base_transform.json')
    with open(world_to_right_robot_path, "r") as f:
        world_to_right_robot_transform = np.array(json.loads(f.read()))

    world_to_camera_transform = np.linalg.inv(camera_to_left_robot_base_transform) @ world_to_left_robot_transform
    # visualize transform with Open3D
    world_in_world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    left_in_world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(
        np.linalg.inv(world_to_left_robot_transform))
    right_in_world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(
        np.linalg.inv(world_to_right_robot_transform))
    camera_in_world = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5).transform(
        np.linalg.inv(world_to_camera_transform))
    o3d.visualization.draw_geometries(
        [world_in_world, left_in_world, right_in_world, camera_in_world])  # world coord system
