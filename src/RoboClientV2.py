# client.py
import socket
import pickle
import numpy as np
import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)
sys.path.append(workspace)
from src.AuboControl import AuboController
from src.EG24BControl import EG24BController
from utils.transform import *
from utils.config import NORM_Z
ROBOTSTATE = {
    'READY_TO_START': 0,
    'GET_ALL_CUBE': 1,
    'FINISH_CUBE0': 2,
    'FINISH_CUBE1': 3,
    'FINISH_CUBE2':4,
    'FINISH_ALL':5,
}
class RoboClient:
    def __init__(self, vision_host='localhost', vision_port=2024,aubo_host='192.168.70.100',aubo_port=8899,gripper_port=None):
        self.vision_host = vision_host
        self.vision_port = vision_port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect((self.vision_host, self.vision_port))
            print('已连接到服务器')
        except Exception as e:
            print(e)
            print('连接服务器失败')
            self.client_socket = None

        self.aubo_host = aubo_host
        self.aubo_port = aubo_port
        self.aubo = AuboController()
        self.aubo.connect(self.aubo_host,self.aubo_port)
        self.aubo.init_tf_tree()

        self.robot_state = ROBOTSTATE['READY_TO_START']
        if gripper_port is not None:
            self.gripper = EG24BController(port_name=gripper_port)

    def send_command(self,command):
        '''
        发送指令
        command: str, 指令
        '''
        self.command = command
        self.client_socket.sendall(self.command.encode())

    def receive_data(self):
        '''
        接收数据
        '''
        data_len_bytes = self.client_socket.recv(4)
        data_len = int.from_bytes(data_len_bytes, byteorder='big')
        print(f'即将接收数据，长度为：{data_len}字节')
        data_received = b''
        while len(data_received) < data_len:
            packet = self.client_socket.recv(4096)
            if not packet:
                break
            data_received += packet
        result = pickle.loads(data_received)
        print('已收到处理后的数据')
        return result
    
    def disconnect(self):
        self.client_socket.close()
        print('已断开连接')

    def get_cube_pose(self,response,id):
        """
        获取物块的位置和姿态
        response: list, 服务器返回的数据
        id: int, 物块的id
        return: 
                pos: np.array, 物块的位置[x,y,z]
                ori: np.array, 物块的姿态[roll,pitch,yaw]
        """
    
        pos = np.zeros(3)
        ori = np.zeros(3)
        for item in response:
            if item[-1] != id:
                continue
            else:
                pos[0] = item[0]
                pos[1] = item[1]
                # pos[2] = NORM_Z
                pos[2] = 0
                ori[2] = np.radians(item[2])
                # ori[2] = 0
                return pos , ori
        
def check_all(response):
    check_list = np.array([0,0,0])
    for item in response:
        check_list[(int)(item[-1])] = 1
    if np.sum(check_list)>=3:
        return True
    else :
        return False


def main():
    # 别问我为什么这个函数这么丑，问就是写到最后不想写了
    import time
    vision_host = 'localhost'  # 使用回传地址
    vision_port = 2024  # 与服务器端保持一致
    aubo_host = '192.168.70.100'
    aubo_port = 8899
    gripper_port = 'COM7'
    traget_pose = []
    client = RoboClient(vision_host, vision_port,aubo_host,aubo_port,gripper_port)
    client.aubo.robot.set_end_max_line_acc(0.1)
    client.aubo.robot.set_end_max_line_velc(0.12)
    
    Z_camera_ground = NORM_Z*1000+16-6.7  #-6.7
    
    init_pos = np.array([-0.43322,-0.131482,0.39424])
    # init_pos = np.array([-0.43322,-0.131482,0.368112])#法兰初始位姿，一定要改！！！
    init_ori = np.array([180*np.pi/180,0,-90*np.pi/180])
    client.aubo.movel_tf(pos=init_pos,ori=init_ori,frame_name='flange_center')
    # camera_init_pos = np.array([-0.54458107,-0.13170145 ,0.38423726])
    # camera_init_pos =np.array([-0.54379159,-0.13207375 ,0.35799774])
    # camera_init_ori = np.array([-180*np.pi/180,0,90*np.pi/180])
    # client.aubo.movel_tf(pos=camera_init_pos,ori=camera_init_ori,frame_name='camera_center')
    T_camera2base =client.aubo.tf_tree.get_transform('camera_center','world')  
    # run this only once
    camera_init_pos, camera_init_ori = client.aubo.tf_tree.transform_to_pose(T_camera2base)
    camera_init_ori = quaternion_to_rpy(np.array(camera_init_ori))
    print("The init pose of camera",camera_init_pos,camera_init_ori)
    
    time.sleep(1)
    if client.robot_state != ROBOTSTATE['READY_TO_START']:
        raise ValueError('初始化失败，请检查连接')

    client.send_command('capture')
    response = client.receive_data()
    if not check_all(response):
        raise ValueError('无法获取全部物块的位置，请检查相机视野')
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)
    cube1_pos_in_camera , cube1_ori_in_camera = client.get_cube_pose(response,1)
    cube2_pos_in_camera , cube2_ori_in_camera = client.get_cube_pose(response,2)\
    
    # modift the pos and ori to the pose in world
    # 将相机坐标旋转180°到工件坐标
    cube1_ori_in_camera[2] += np.pi
    cube2_ori_in_camera[2] += np.pi
    cube1_ori_in_camera = rpy_to_quaternion(np.array(cube1_ori_in_camera))
    cube2_ori_in_camera = rpy_to_quaternion(np.array(cube2_ori_in_camera))
    cube1_pos_in_camera , cube1_ori_in_camera = client.aubo.tf_tree.transform_pose(cube1_pos_in_camera,cube1_ori_in_camera,'camera_center','world')
    cube2_pos_in_camera , cube2_ori_in_camera = client.aubo.tf_tree.transform_pose(cube2_pos_in_camera,cube2_ori_in_camera,'camera_center','world')
    cube1_ori_in_camera = quaternion_to_rpy(np.array(cube1_ori_in_camera))
    cube2_ori_in_camera = quaternion_to_rpy(np.array(cube2_ori_in_camera))

    # get the pos of cube 0
    client.aubo.movel_relative(cube0_pos_in_camera,cube0_ori_in_camera,frame_name='camera_center')
    time.sleep(1)#等待运动稳定
    client.send_command('capture')
    response = client.receive_data()
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)
    client.aubo.movel_relative(cube0_pos_in_camera,cube0_ori_in_camera,"camera_center")
    time.sleep(1)#等待运动稳定
   
    client.send_command('capture')
    response = client.receive_data()
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)
    cube0_pos_in_camera[2] = NORM_Z -0.015#将物块1的z坐标归一化

    # modift the angle
    client.send_command('modify_angle0')
    modify_angle = client.receive_data()
    cube0_ori_in_camera[2] +=modify_angle

    # transform to cuber_refer
    cube0_ori_in_camera[2] +=np.pi
    cube0_ori_in_camera = rpy_to_quaternion(np.array(cube0_ori_in_camera))
    pos, ori = client.aubo.tf_tree.transform_pose(cube0_pos_in_camera,cube0_ori_in_camera,'camera_center','world')
    home_pos = cube0_pos_in_camera.copy()
    home_pos[2] = Z_camera_ground/1000 # 是标定值，可以微调查看效果
    home_ori = cube0_ori_in_camera.copy()
    home_pos,home_ori = client.aubo.tf_tree.transform_pose(home_pos,home_ori,'camera_center','world')
    client.aubo.tf_tree.add_node("home","world",home_pos,home_ori)
    ori = quaternion_to_rpy(np.array(ori))
    traget_pose.append([pos,ori])
    
    

    # get the pos of cube 1
    client.aubo.movel_tf(pos=cube1_pos_in_camera,ori=cube1_ori_in_camera,frame_name='cube_refer')
    time.sleep(1)#等待运动稳定
    
    client.send_command('capture')
    response = client.receive_data()
    cube1_pos_in_camera , cube1_ori_in_camera = client.get_cube_pose(response,1)
    client.aubo.movel_relative(cube1_pos_in_camera,cube1_ori_in_camera,"camera_center")
    time.sleep(1)#等待运动稳定
   
    client.send_command('capture')
    response = client.receive_data()
    cube1_pos_in_camera , cube1_ori_in_camera = client.get_cube_pose(response,1)
    cube1_pos_in_camera[2] = NORM_Z#将物块1的z坐标归一化

    # modift the angle
    client.send_command('modify_angle1')
    modify_angle = client.receive_data()
    cube1_ori_in_camera[2] +=modify_angle
    # 将相机坐标旋转180°到工件坐标
    cube1_ori_in_camera[2] +=np.pi
    cube1_ori_in_camera = rpy_to_quaternion(np.array(cube1_ori_in_camera))
    pos, ori = client.aubo.tf_tree.transform_pose(cube1_pos_in_camera,cube1_ori_in_camera,'camera_center','world')
    ori = quaternion_to_rpy(np.array(ori))
    traget_pose.append([pos,ori])

    # get the pos of cube 2
    client.aubo.movel_tf(pos=cube2_pos_in_camera,ori=cube2_ori_in_camera,frame_name='cube_refer')
    time.sleep(1)#等待运动稳定
    client.send_command('capture')
    response = client.receive_data()
    cube2_pos_in_camera , cube2_ori_in_camera = client.get_cube_pose(response,2)
    client.aubo.movel_relative(cube2_pos_in_camera,cube2_ori_in_camera,"camera_center")
    time.sleep(1)#等待运动稳定
    client.send_command('capture')
    response = client.receive_data()
    cube2_pos_in_camera , cube2_ori_in_camera = client.get_cube_pose(response,2)
    cube2_pos_in_camera[2] = NORM_Z-0.002#将物块1的z坐标归一化
    # 将相机坐标旋转180°到工件坐标
    cube2_ori_in_camera[2] +=np.pi
    cube2_ori_in_camera = rpy_to_quaternion(np.array(cube2_ori_in_camera))
    pos, ori = client.aubo.tf_tree.transform_pose(cube2_pos_in_camera,cube2_ori_in_camera,'camera_center','world')
    ori = quaternion_to_rpy(np.array(ori))
    # # traget_pose.append([pos,ori])
    traget_pose.append([pos,ori])


    # take 1 to 0
    safe_dist = np.array([0,0,0.05])
    client.aubo.movel_tf(traget_pose[1][0]+safe_dist,traget_pose[1][1],'gripper_center')
    client.aubo.movel_tf(traget_pose[1][0],traget_pose[1][1],'gripper_center')
    client.gripper.close_gripper(speed=500, force=100)
    time.sleep(0.5)
    client.aubo.movel_relative(-safe_dist,np.array([0,0,0]),'gripper_center')
    client.aubo.movel_tf(traget_pose[0][0]+safe_dist,traget_pose[0][1],'gripper_center')
    client.aubo.robot.set_end_max_line_acc(0.08)
    client.aubo.robot.set_end_max_line_velc(0.08)
    client.aubo.movel_tf(traget_pose[0][0],traget_pose[0][1],'gripper_center')
    # time.sleep(1)
    client.gripper.open_gripper(speed=500)
    time.sleep(0.5)
    client.aubo.robot.set_end_max_line_acc(0.1)
    client.aubo.robot.set_end_max_line_velc(0.15)
    client.aubo.movel_tf(traget_pose[0][0]+safe_dist,traget_pose[0][1],'gripper_center')
    client.aubo.movel_tf(traget_pose[2][0]+safe_dist,traget_pose[2][1],'gripper_center')
    client.aubo.movel_tf(traget_pose[2][0],traget_pose[2][1],'gripper_center')
    # time.sleep(1)
    client.gripper.close_gripper(speed=500, force=100)
    time.sleep(0.8)
    client.aubo.movel_relative(-safe_dist,np.array([0,0,0]),'gripper_center')
    # take 2 to 1
    # pos_cube2cube_refer = traget_pose[0][0]
    
    Z_cube2_ground = 34.7 # 全是理论值，理论上无需调整
    pos_slot2home = np.array([-0.01,0.0,-Z_cube2_ground/1000])
    ori_slot2home = np.array([-15*np.pi/180,0,0])
    ori_slot2home = rpy_to_quaternion(ori_slot2home)
    slot_offset = np.array([0.0,0.0,0.003])
    
    client.aubo.tf_tree.add_node("slot","home",pos_slot2home,ori_slot2home)
    client.aubo.tf_tree.add_node("slot_offset","slot",-slot_offset,np.array([0,0,0,1]))
    client.aubo.tf_tree.add_node("safe_slot","slot",-safe_dist,np.array([0,0,0,1]))
    T_slot2world = client.aubo.tf_tree.get_transform("slot_offset","world")
    slot_pos,slot_ori = client.aubo.tf_tree.transform_to_pose(T_slot2world)
    slot_ori = quaternion_to_rpy(np.array(slot_ori))
    T_safe_slot2world = client.aubo.tf_tree.get_transform("safe_slot","world")
    safe_slot_pos,safe_slot_ori = client.aubo.tf_tree.transform_to_pose(T_safe_slot2world)
    safe_slot_ori = quaternion_to_rpy(np.array(safe_slot_ori))

    client.aubo.movel_tf(safe_slot_pos,safe_slot_ori,'gripper_center')
    # time.sleep(1)
    client.aubo.robot.set_end_max_line_acc(0.05)
    client.aubo.robot.set_end_max_line_velc(0.08)
    client.aubo.movel_tf(slot_pos,slot_ori,'gripper_center')
    # time.sleep(1)
    client.gripper.open_gripper(speed=500)
    time.sleep(0.5)
    client.aubo.robot.set_end_max_line_acc(0.1)
    client.aubo.robot.set_end_max_line_velc(0.15)
    client.aubo.movel_tf(pos=camera_init_pos,ori=camera_init_ori,frame_name='camera_center')
    client.aubo.tf_tree.delete_node("safe_slot")
    client.aubo.tf_tree.delete_node("slot_offset")
    client.aubo.tf_tree.delete_node("slot")

    client.disconnect()
    client.aubo.disconnect()


if __name__ == '__main__':
    main()