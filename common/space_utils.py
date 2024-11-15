import numpy as np
from typing import Tuple
import transforms3d as t3d
from geometry_msgs.msg import Pose

def ros_pose_to_4x4matrix(pose: Pose) -> np.ndarray:
    # Convert ROS Pose message to 4x4 transformation matrix
    mat = np.eye(4)
    quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    rot_mat = t3d.quaternions.quat2mat(quat)
    mat[:3, :3] = rot_mat
    mat[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])
    return mat

def ros_pose_to_6d_pose(pose: Pose) -> np.ndarray:
    # convert ROS Pose message to 6D pose (x, y, z, r, p, y)
    quat = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    euler = t3d.euler.quat2euler(quat)
    trans = np.array([pose.position.x, pose.position.y, pose.position.z])
    return np.concatenate([trans, euler])

def pose_6d_to_pose_7d(pose: np.ndarray) -> np.ndarray:
    # convert 6D pose (x, y, z, r, p, y) to 7D pose (x, y, z, qw, qx, qy, qz)
    quat = t3d.euler.euler2quat(pose[3], pose[4], pose[5])
    return np.concatenate([pose[:3], quat])

def pose_7d_to_pose_6d(pose: np.ndarray) -> np.ndarray:
    # convert 7D pose (x, y, z, qw, qx, qy, qz) to 6D pose (x, y, z, r, p, y)
    quat = pose[3:]
    euler = t3d.euler.quat2euler(quat)
    return np.concatenate([pose[:3], euler])

def pose_7d_to_4x4matrix(pose: np.ndarray) -> np.ndarray:
    # convert 7D pose (x, y, z, qw, qx, qy, qz) to 4x4 transformation matrix
    mat = np.eye(4)
    mat[:3, :3] = t3d.quaternions.quat2mat(pose[3:])
    mat[:3, 3] = pose[:3]
    return mat

def pose_6d_to_4x4matrix(pose: np.ndarray) -> np.ndarray:
    # convert 6D pose (x, y, z, r, p, y) to 4x4 transformation matrix
    mat = np.eye(4)
    quat = t3d.euler.euler2quat(pose[3], pose[4], pose[5])
    mat[:3, :3] = t3d.quaternions.quat2mat(quat)
    mat[:3, 3] = pose[:3]
    return mat

def matrix4x4_to_pose_6d(mat: np.ndarray) -> np.ndarray:
    # convert 4x4 transformation matrix to 6D pose (x, y, z, r, p, y)
    quat = t3d.quaternions.mat2quat(mat[:3, :3])
    euler = t3d.euler.quat2euler(quat)
    trans = mat[:3, 3]
    return np.concatenate([trans, euler])

def ortho6d_to_rotation_matrix(ortho6d: np.ndarray) -> np.ndarray:
    """
    Compute rotation matrix from ortho6d representation
    """
    x_raw = ortho6d[:, 0:3]  # batch * 3
    y_raw = ortho6d[:, 3:6]  # batch * 3
    x = normalize_vector(x_raw)  # batch * 3
    z = np.cross(x, y_raw)  # batch * 3
    z = normalize_vector(z)  # batch * 3
    y = np.cross(z, x)  # batch * 3

    x = x[:, :, np.newaxis]
    y = y[:, :, np.newaxis]
    z = z[:, :, np.newaxis]

    matrix = np.concatenate((x, y, z), axis=2)  # batch * 3 * 3
    return matrix

def normalize_vector(v: np.ndarray) -> np.ndarray:
    """
    Normalize a vector (batch * 3)
    """
    v_mag = np.linalg.norm(v, axis=1, keepdims=True)  # batch * 1
    v_mag = np.maximum(v_mag, 1e-8)
    v = v / v_mag
    return v

def transform_point_cloud(pcd: np.ndarray, transform_matrix: np.ndarray) -> np.ndarray:
    """
    Transform a point cloud with 4x4 transform_matrix
    Parameters
    ----------
    pcd: (N, 6) or (N, 3) ndarray
    transform_matrix: (4, 4) ndarray
    """
    if pcd.shape[1] == 3: # (x, y, z)
        transformed_xyz = np.matmul(transform_matrix[:3, :3], pcd.T).T + transform_matrix[:3, 3]
        return transformed_xyz
    elif pcd.shape[1] == 6:  # (x, y, z, r, p, y)
        transformed_xyz = np.matmul(transform_matrix[:3, :3], pcd[:, :3].T).T + transform_matrix[:3, 3]
        return np.concatenate([transformed_xyz, pcd[:, 3:]], axis=1)
    else:
        raise NotImplementedError

