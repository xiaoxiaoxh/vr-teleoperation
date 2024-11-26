import numpy as np
import transforms3d as t3d
import json

def unity2zup_right_frame(pos_quat: np.ndarray):
        pos_quat = np.array(pos_quat)
        assert(pos_quat.shape[0]==7)
        pos_quat*=np.array([1,-1,1,1,-1,1,-1])
        rot_mat = t3d.quaternions.quat2mat(pos_quat[3:])
        pos_vec = pos_quat[:3]
        T=np.eye(4)
        T[:3,:3]= rot_mat
        T[:3,3]=pos_vec
        fit_mat = t3d.euler.axangle2mat([0,1,0],np.pi/2)
        fit_mat = fit_mat@t3d.euler.axangle2mat([0,0,1],-np.pi/2)
        target_rot_mat=fit_mat@rot_mat
        target_pos_vec=fit_mat@pos_vec
        target = np.array(target_pos_vec.tolist()+t3d.quaternions.mat2quat(target_rot_mat).tolist())
        return target
data = dict()
with open("./FingerData.json",'r') as f:
        data = json.loads(f.read())["data"]

final_result = []
for frame in data:
        frame_result = dict()
        frame_result["time"] = frame["time"]

        udata = [frame["Thumb"]["x"],
                    frame["Thumb"]["y"],
                    frame["Thumb"]["z"],
                    1,0,0,0]
        frame_result["Thumb"] = unity2zup_right_frame(udata)[:3].tolist()

        udata = [frame["Index"]["x"],
                    frame["Index"]["y"],
                    frame["Index"]["z"],
                    1,0,0,0]
        frame_result["Index"] = unity2zup_right_frame(udata)[:3].tolist()

        udata = [frame["Middle"]["x"],
                    frame["Middle"]["y"],
                    frame["Middle"]["z"],
                    1,0,0,0]
        frame_result["Middle"] = unity2zup_right_frame(udata)[:3].tolist()

        udata = [frame["Ring"]["x"],
                    frame["Ring"]["y"],
                    frame["Ring"]["z"],
                    1,0,0,0]
        frame_result["Ring"] = unity2zup_right_frame(udata)[:3].tolist()

        udata = [frame["Pinky"]["x"],
                    frame["Pinky"]["y"],
                    frame["Pinky"]["z"],
                    1,0,0,0]
        frame_result["Pinky"] = unity2zup_right_frame(udata)[:3].tolist()
        final_result.append(frame_result)

with open("./ConvertData.json",'w') as f:
        f.write(json.dumps(final_result))

                
            
