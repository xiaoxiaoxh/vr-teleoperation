import cv2
import numpy as np
import open3d as o3d
from typing import List

def visualize_rgb_image(image: np.ndarray, name: str = 'Image'):
    cv2.imshow(name, image)
    cv2.waitKey(1)
    # cv2.destroyAllWindows()

def visualize_pcd_from_numpy(xyz_rgb: np.ndarray):
    """
    Visualize point cloud data
    :param xyz_rgb: numpy array of shape (N, 6) (x, y, z, r, g, b)
    """
    raw_pcd = o3d.geometry.PointCloud()
    raw_pcd.points = o3d.utility.Vector3dVector(xyz_rgb[:, :3])
    raw_pcd.colors = o3d.utility.Vector3dVector(xyz_rgb[:, 3:6]) # [0., 1.]
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    o3d.visualization.draw_geometries([raw_pcd, coord])