import os
import sys
import cv2
import time
import numpy as np
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)

sys.path.append(workspace)

from utils.transform import *
from src.MVScontrol import *
from src.AuboControl import AuboController


def MVSInit():
    # 获得设备信息
    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch: 枚举设备 | en:Enum device
    # nTLayerType[IN] 枚举传输层 ，pstDevList[OUT] 设备列表
    Enum_device(tlayerType, deviceList)

    # 获取相机和图像数据缓存区
    cam, data_buf, nPayloadSize = enable_device(0,deviceList)  # 选择第一个设备
    return cam, data_buf, nPayloadSize

def capture_frame(cam,data_buf,nPayloadSize,name,targrt_dir='./',show = False):
    os.makedirs(targrt_dir,exist_ok=True)
    prefiex = "capture"
    file_path = os.path.join(targrt_dir,prefiex+name+'.jpg')
    image = get_image(cam,data_buf, nPayloadSize)
    cv2.imwrite(file_path,image)
    if show:
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.imshow("image", image)
        if cv2.waitKey(500) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            
def get_calib_data(aubo,cam,data_buf,nPayloadSize):
    
    aubo.init_tf_tree()
    trans = [ 0.0,0.0,0.421]
    rotation = [0.0,0.0,0.0,1]
    aubo.tf_tree.add_node("temp","camera_center",trans,rotation)
    temp_pos ,temp_ori = aubo.get_pose("temp")
    temp_ori = quaternion_to_rpy(np.array(temp_ori))
    waypoint = aubo.get_current_waypoint()
    if waypoint is not None:
        pos = waypoint['pos']
        ori = waypoint['ori']
        ori = quaternion_to_rpy(np.array(ori))
    else:
        raise ValueError("waypoint is None, cannot proceed.")
    calib_data = []
    delta_pos = [0.02,0.03,0.0]
    delta_ori = [20*np.pi/180,30*np.pi/180,25*np.pi/180]
    count = 0
    for i in range(-1,2):
        # flag_x = 1-i
        for j in range(-1,2):
            # flag_y = 1-j
            if waypoint is not None:
                pos_target = pos.copy()
                pos_target[0] += i*delta_pos[0]
                pos_target[1] += j*delta_pos[1]
                aubo.movel_tf(pos_target,ori,frame_name="flange_center")
                time.sleep(3)
                aubo.update_flange_center()
                T = aubo.tf_tree.get_transform("flange_center","world")
                calib_data.append(T)
                capture_frame(cam,data_buf,nPayloadSize,f'{count}',targrt_dir='../dataset/calib',show = True)
                count += 1
            else:
                raise ValueError("waypoint is None, cannot proceed.")
    aubo.movel(pos,ori)
    for i in range(3):
        for j in range(2):
            flag = 1-2*j
            if waypoint is not None:
                ori_target = temp_ori.copy()
                ori_target[i] += flag*delta_ori[i]
                aubo.movel_tf(temp_pos,ori_target,frame_name="temp")
                time.sleep(3)
                aubo.update_flange_center()
                T = aubo.tf_tree.get_transform("flange_center","world")
                calib_data.append(T)
                capture_frame(cam,data_buf,nPayloadSize,f'{count}',targrt_dir='../dataset/calib',show = True)
                count += 1
            else:
                raise ValueError("waypoint is None, cannot proceed.")
    aubo.movel(pos,ori)
    for i in range(2):
        flagx = 1-2*i
        for j in range(2):
            flagy = 1-2*j
            if waypoint is not None:
                ori_target = temp_ori.copy()
                ori_target[0] += flagx*delta_ori[0]
                ori_target[1] += flagy*delta_ori[1]
                aubo.movel_tf(temp_pos,ori_target,frame_name="temp")
                time.sleep(3)
                aubo.update_flange_center()
                T = aubo.tf_tree.get_transform("flange_center","world")
                calib_data.append(T)
                capture_frame(cam,data_buf,nPayloadSize,f'{count}',targrt_dir='../dataset/calib',show = True)
                count += 1
    aubo.movel(pos,ori)
     # 将 calib_data 列表存储为 NumPy 文件
    for i in range(2):
        flagx = 1-2*i
        for j in range(2):
            flagy = 1-2*j
            for k in range(2):
                flagz = 1-2*k
                if waypoint is not None:
                    ori_target = temp_ori.copy()
                    ori_target[0] += flagx*delta_ori[0]
                    ori_target[1] += flagy*delta_ori[1]
                    ori_target[2] += flagz*delta_ori[2]
                    aubo.movel_tf(temp_pos,ori_target,frame_name="temp")
                    time.sleep(3)
                    aubo.update_flange_center()
                    T = aubo.tf_tree.get_transform("flange_center","world")
                    calib_data.append(T)
                    capture_frame(cam,data_buf,nPayloadSize,f'{count}',targrt_dir='../dataset/calib',show = True)
                    count += 1
    aubo.movel(pos,ori)
    aubo.tf_tree.delete_node("temp")
    np.save('flange2base.npy', np.array(calib_data))

    print("Calibration data has been saved to calib_data.npy")
    return calib_data
def get_dataset(num,pos,ori,aubo,cam,data_buf,nPayloadSize):
     delta_pos = [0.05,0.05,0.0]
     for k in range(num):
        for i in range(-1,2):
            # flag_x = 1-i
            for j in range(-1,2):
                # flag_y = 1-j
                if waypoint is not None:
                    pos_target = pos.copy()
                    pos_target[0] += i*delta_pos[0]
                    pos_target[1] += j*delta_pos[1]
                    aubo.movel(pos_target,ori)
                    capture_frame(cam,data_buf,nPayloadSize,f'{i+1+3*k}_{j+1+3*k}_3',targrt_dir='../dataset/all',show = True)
                else:
                    raise ValueError("waypoint is None, cannot proceed.")
                # raise ValueError("joint_radian is None, cannot proceed.")
        aubo.movel(pos,ori)
        time.sleep(5)
def get_circle_data(num,aubo:AuboController,cam,data_buf,nPayloadSize):
    delta_pos = np.array([0,0,0])
    delta_ori = np.array([0.0,0.0,5*np.pi/180])
    original_pos,original_ori =aubo.get_pose("camera_center")
    original_ori = quaternion_to_rpy(np.array(original_ori))
    for k in range(num):
        aubo.movel_tf(original_pos,original_ori,frame_name="camera_center")
        for i in range(9):
                aubo.movel_relative(delta_pos,delta_ori,frame_name="camera_center")
                capture_frame(cam,data_buf,nPayloadSize,f'{i+1+9*k}_1',targrt_dir='../dataset/circle',show = True)
        aubo.movel(pos,ori)
        time.sleep(5)
if __name__ == "__main__":
    ip = '192.168.70.100'
    port = 8899
    cam, data_buf, nPayloadSize = MVSInit()

    aubo = AuboController()
    aubo.connect(ip,port)
    aubo.robot.set_end_max_line_acc(0.05)
    aubo.robot.set_end_max_line_velc(0.05)
    waypoint = aubo.get_current_waypoint()
    if waypoint is not None:
        joint_radian = waypoint['joint']
        pos = waypoint['pos']
        ori = waypoint['ori']
        ori = quaternion_to_rpy(np.array(ori))
    else:
        raise ValueError("waypoint is None, cannot proceed.")
    
    # delta_pos = [0.05,0.05,0.0]
    # deltal_ori = [0.0,10*np.pi/180,0.0]
    # get_dataset(1,pos,ori,aubo,cam,data_buf,nPayloadSize)
    get_calib_data(aubo,cam,data_buf,nPayloadSize)
    # get_circle_data(5,aubo,cam,data_buf,nPayloadSize)
    # capture_frame(cam,data_buf,nPayloadSize,'template0',targrt_dir='../dataset/template',show = True)
    aubo.disconnect()
    close_device(cam, data_buf)
