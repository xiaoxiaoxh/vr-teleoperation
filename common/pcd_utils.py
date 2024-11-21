import pytorch3d.ops as torch3d_ops
import torch
import open3d as o3d
import numpy as np

def farthest_point_sampling(points, num_points=1024, use_cuda=True):
    K = [num_points]
    if use_cuda:
        points = torch.from_numpy(points).cuda()
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.cpu().numpy()
        indices = indices.cpu().numpy()
    else:
        points = torch.from_numpy(points)
        sampled_points, indices = torch3d_ops.sample_farthest_points(points=points.unsqueeze(0), K=K)
        sampled_points = sampled_points.squeeze(0)
        sampled_points = sampled_points.numpy()
        indices = indices.numpy()

    return sampled_points, indices

def random_sample_points(points: np.ndarray, random_sample_num_points: int):
    if points.shape[0] > random_sample_num_points:
        all_idxs = np.arange(points.shape[0])
        rs = np.random.RandomState()
        selected_idxs = rs.choice(all_idxs, size=random_sample_num_points, replace=False)
        sel_points = points[selected_idxs, :]
    else:
        sel_points = points
    return sel_points

def random_sample_pcd(pcd: o3d.geometry.PointCloud,
                      random_sample_num_points: int,
                      return_pcd: bool = True):
    '''
    pcd: an o3d color pointcloud object
    random_sample_num_points: number of points to be randomly sampled
    return_pcd: if True, return the pcd object; if False, return points and colors

    '''
    # random select fixed number of points
    pts_xyz = np.asarray(pcd.points)
    pts_rgb = np.asarray(pcd.colors)
    if pts_xyz.shape[0] > random_sample_num_points:
        all_idxs = np.arange(pts_xyz.shape[0])
        rs = np.random.RandomState()
        selected_idxs = rs.choice(all_idxs, size=random_sample_num_points, replace=False)
        pc_xyz_slim = pts_xyz[selected_idxs, :]
        pc_rgb_slim = pts_rgb[selected_idxs, :]
        if return_pcd:
            pcd.points = o3d.utility.Vector3dVector(pc_xyz_slim)
            pcd.colors = o3d.utility.Vector3dVector(pc_rgb_slim)
        else:
            return pc_xyz_slim, pc_rgb_slim

    if return_pcd:
        return pcd
    else:
        return pts_xyz, pts_rgb