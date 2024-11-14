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
    
def wirte2yaml(T,output_dir):
    import yaml
    os.makedirs(output_dir, exist_ok=True)
    # 自定义 Dumper 以处理 numpy 数组的换行显示
    class MyDumper(yaml.Dumper):
        def increase_indent(self, flow=False, indentless=False):
            return super(MyDumper, self).increase_indent(flow=True, indentless=False)
    
    dict_data = {
        "T_gripper1to2": T.tolist()
    }

    with open(f"{output_dir}/camera_config.yaml", "w") as f:
        yaml.dump(dict_data, f,default_flow_style=True,sort_keys=False,Dumper=MyDumper)   
       
    print("Camera config matrix  have been saved to camera_config.yaml")

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
    
    Z_camera_ground = NORM_Z*1000+16 - 7# 16成像平面到地面的高度，5是夹爪中心到地面的距离
    
    # init the robot pose
    init_pos = np.array([-0.43322,-0.131482,0.39424])
    # init_pos = np.array([-0.43322,-0.131482,0.368112])#法兰初始位姿，一定要改！！！
    init_ori = np.array([180*np.pi/180,0,-90*np.pi/180])
    client.aubo.movel_tf(pos=init_pos,ori=init_ori,frame_name='flange_center')
    
    
    time.sleep(1)
    if client.robot_state != ROBOTSTATE['READY_TO_START']:
        raise ValueError('初始化失败，请检查连接')

    client.send_command('capture')
    response = client.receive_data()
    # if not check_all(response):
    #     raise ValueError('无法获取全部物块的位置，请检查相机视野')
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)
    
    # get the pos of cube 0
    client.aubo.movel_relative(cube0_pos_in_camera,cube0_ori_in_camera,frame_name='camera_center')
    time.sleep(1)#等待运动稳定
    client.send_command('capture')
    response = client.receive_data()
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)
    client.aubo.movel_relative(cube0_pos_in_camera,cube0_ori_in_camera,frame_name='camera_center')
    time.sleep(1)#等待运动稳定
    client.send_command('capture')
    response = client.receive_data()
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)
    # modift the pos and ori to the pose in world
    # 将相机坐标旋转180°到工件坐标
    cube0_ori_in_camera[2] += np.pi
    # 将相机坐标转换到世界坐标
    cube0_pos_in_camera[2] = Z_camera_ground/1000 # 调整此值让夹爪抓到刚好接触地面
    cube0_ori_in_camera = rpy_to_quaternion(np.array(cube0_ori_in_camera))
    cube0_pos_in_camera , cube0_ori_in_camera = client.aubo.tf_tree.transform_pose(cube0_pos_in_camera,cube0_ori_in_camera,'camera_center','world')
    cube0_ori_in_camera = quaternion_to_rpy(np.array(cube0_ori_in_camera))

    # 移动到物块0的位置，实现硬定位
    safe_dist = np.array([0,0,0.05])
    client.aubo.movel_tf(cube0_pos_in_camera + safe_dist,cube0_ori_in_camera,'gripper_center')
    client.aubo.movel_tf(cube0_pos_in_camera,cube0_ori_in_camera,'gripper_center')
    time.sleep(1)
    client.gripper.close_gripper(speed=500, force=100)
    time.sleep(1)
    T_gripper2world_1 = client.aubo.tf_tree.get_transform("gripper_center","world")
    client.gripper.open_gripper(speed=500)
    time.sleep(1)
    # 旋转90度再次抓取
    client.aubo.movel_relative(-safe_dist,np.array([0,0,0]),'gripper_center')
    
    client.aubo.movel_relative(np.array([0,0,0]),np.array([0,0,np.pi/2]),'gripper_center')
    client.aubo.movel_relative(safe_dist,np.array([0,0,0]),'gripper_center')
    time.sleep(1)
    client.gripper.close_gripper(speed=500, force=100)
    time.sleep(1)
    client.gripper.open_gripper(speed=500)
    time.sleep(1)

    # 抬升到初始位置进行视觉定位，移动物块到视野中心
    client.aubo.movel_relative(-safe_dist,np.array([0,0,0]),'gripper_center')
    client.aubo.movel_tf(init_pos,init_ori,'flange_center')
    time.sleep(1)

    # get the pos of cube 0
    client.send_command('capture')
    response = client.receive_data()
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)
    client.aubo.movel_relative(cube0_pos_in_camera,cube0_ori_in_camera,"camera_center")
    time.sleep(1)#等待运动稳定

    client.send_command('capture')
    response = client.receive_data()
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)
    client.aubo.movel_relative(cube0_pos_in_camera,cube0_ori_in_camera,"camera_center")
    time.sleep(1)#等待运动稳定
    
    client.send_command('capture')
    response = client.receive_data()
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)
    client.aubo.movel_relative(cube0_pos_in_camera,cube0_ori_in_camera,"camera_center")
    time.sleep(1)#等待运动稳定
    # 多次循环，保证无差定位

    # 记录此时夹爪位置，输出相对变换关系。
    T_gripper2world_2 = client.aubo.tf_tree.get_transform("gripper_center","world")
    T_gripper1to2 = np.linalg.inv(T_gripper2world_2) @ T_gripper2world_1

    print("The relative transform between gripper1 and gripper2 is:",T_gripper1to2)

    v_trans = T_gripper1to2[:3,3]
    v_rot = T_gripper1to2[:3,:3]
    print("The translation between gripper1 and gripper2 is:",v_trans)
    print("The rotation between gripper1 and gripper2 is:",R.from_matrix(v_rot).as_euler('xyz', degrees=True))

    output_dir = f"{current_dir}/calib_data/output"
    wirte2yaml(T_gripper1to2,output_dir)

    client.disconnect()
    client.aubo.disconnect()


if __name__ == '__main__':
    main()