import open3d as o3d
import transforms3d as t3d
import json
import numpy as np

data = dict()
with open("./ConvertData_z-front_x-rotate_y-up_xyzqwqxqyqz.json",'r') as f:
        data = json.loads(f.read())

data = data[100]
names = ["Thumb","Index","Middle","Ring","Pinky"]

pivots = []
pivots.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0]))
for n in names:
    pivot= o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    T = np.eye(4)
    T[:3, :3] = t3d.quaternions.quat2mat(np.array(data[n])[3:])
    T[:3, 3] = np.array(data[n])[:3]
    pivot.transform(np.linalg.inv(T))
    pivots.append(pivot)

o3d.visualization.draw_geometries(pivots)