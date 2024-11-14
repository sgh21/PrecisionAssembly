import os
import sys
import numpy as np
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)

sys.path.append(workspace)

from utils.CalibrateHelper import Calibrator
from utils.TsaiLenzMethod import Tsai_HandEye, calculate_homogeneous_transform
from scipy.spatial.transform import Rotation as R



def Intrinsic_Calibrate(img_dir, shape_inner_corner, size_grid,output_dir=None):
    # create calibrator
    calibrator = Calibrator(img_dir, shape_inner_corner, size_grid)
    
    # calibrate the camera
    mat_intri, coff_dis,v_rot,v_trans = calibrator.calibrate_camera()

    # # dedistort and save the dedistortion result
    if output_dir is not None:
        print("Dedistorting...")
        save_dir = f"{output_dir}/IR_dedistortion"
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        calibrator.dedistortion(save_dir)
        np.save(f"{output_dir}/intrinsic.npy", mat_intri)
        np.save(f"{output_dir}/distortion.npy", coff_dis)
        np.save(f"{output_dir}/rotation.npy", v_rot)
        np.save(f"{output_dir}/translation.npy",v_trans)
    return mat_intri, coff_dis, v_rot, v_trans

def HandEye_Calibrate(v_rot,v_trans,T_flange2base):

    ## 构造方程AX=XB
    # T_flange2base = np.load(f"{data_dir}/flange2base.npy")
    if  T_flange2base.shape[0]!= v_trans.shape[0] or \
        T_flange2base.shape[0]!= v_rot.shape[0]:
        raise ValueError("The number of data points does not match!")
    T_calibboard2camera = np.zeros((T_flange2base.shape[0], 4, 4))
    for i in range(T_flange2base.shape[0]):
        T_calibboard2camera[i], _ = calculate_homogeneous_transform(v_rot[i], v_trans[i])
    
    T_A = np.zeros((T_flange2base.shape[0]-1, 4, 4))
    T_B = np.zeros((T_flange2base.shape[0]-1, 4, 4))
    for i in range(T_flange2base.shape[0]-1):
        T_A[i] = np.linalg.inv(T_flange2base[i+1]) @ T_flange2base[i]  
        T_B[i] = T_calibboard2camera[i+1] @ np.linalg.inv(T_calibboard2camera[i])
    T = Tsai_HandEye(T_A, T_B)
    
    return T
    
def wirte2yaml(mat_intri, coff_dis,T,output_dir):
    import yaml
    os.makedirs(output_dir, exist_ok=True)
    # 自定义 Dumper 以处理 numpy 数组的换行显示
    class MyDumper(yaml.Dumper):
        def increase_indent(self, flow=False, indentless=False):
            return super(MyDumper, self).increase_indent(flow=True, indentless=False)
    
    dict_data = {
        "distortion_coefficients": coff_dis.tolist(),
        "intrinsic": mat_intri.tolist(),
        "extrinsic": T.tolist()
    }

    with open(f"{output_dir}/camera_config.yaml", "w") as f:
        yaml.dump(dict_data, f,default_flow_style=True,sort_keys=False,Dumper=MyDumper)   
       
    print("Camera config matrix  have been saved to camera_config.yaml")

if __name__ == "__main__":
    img_dir = "./calib_data/calib_new_biaoding_zhijia"
    output_dir = f"{img_dir}/output"
    
    shape_inner_corner = (33, 32)
    size_grid = 5/1000
    trans_num = 9 
    # calibrate the camera
    mat_intri, coff_dis,v_rot,v_trans = Intrinsic_Calibrate(img_dir, shape_inner_corner, size_grid)
    
    # v_trans = v_trans/1000
   
    # HandEye calibration
    norm_z = 0
    for i in range(trans_num):
        norm_z +=v_trans[i][2]/trans_num
    print("The NORM_Z should be about:",norm_z)
    T_flange2base = np.load(f"{img_dir}/flange2base.npy")[9:]
    indices = []
    
    
    # T_flange2base = np.zeros((v_rot.shape[0],4,4))
    # for i in range(v_rot.shape[0]):
    #     if i < 11:  # 20
    #         T_flange2base[i] = T_temp[i]
    #     else:
    #         T_flange2base[i] = T_temp[i+1]
    # # intrinsics 
    # mat_intri = np.load(f"{img_dir}/mat_intri.npy")
    # coff_dis = np.load(f"{img_dir}/coff_dist.npy")
    # v_rot = np.load(f"{img_dir}/v_rot.npy")
    # # print(v_rot.shape)
    # v_trans = np.load(f"{img_dir}/v_trans.npy")
    # # v_rot = np.delete(v_rot, indices, axis=0)
    # # v_trans = np.delete(v_trans, indices, axis=0)
    # # T_flange2base = np.delete(T_flange2base, indices, axis=0)
    # T = HandEye_Calibrate(np.array(v_rot),np.array(v_trans,),T_flange2base)
    
    # rotation = T[:3,:3]
    # ori = R.from_matrix(rotation).as_euler('xyz', degrees=True)
    # print("Camera calibration result:\n",mat_intri, coff_dis)
    # print("HandEye calibration result:\n",ori)
    # print("HandEye calibration result:\n",T)
    # # write to yaml
    # wirte2yaml(mat_intri, coff_dis,T,output_dir)
   
    