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
    gripper_port = 'COM4'
    traget_pose = []
    client = RoboClient(vision_host, vision_port,aubo_host,aubo_port,gripper_port)
    client.aubo.robot.set_end_max_line_acc(0.1)
    client.aubo.robot.set_end_max_line_velc(0.12)
    
    Z_camera_ground = NORM_Z*1000+16-6.7
    
    init_pos = np.array([-0.43322,-0.131482,0.39424])
    # init_pos = np.array([-0.43322,-0.131482,0.368112])#法兰初始位姿，一定要改！！！
    init_ori = np.array([180*np.pi/180,0,-90*np.pi/180])
    client.aubo.movel_tf(pos=init_pos,ori=init_ori,frame_name='flange_center')

    # camera_init_pos = np.array([-0.54458107,-0.13170145 ,0.38423726])  # 相机相对于基座的初始pos
    # # camera_init_pos =np.array([-0.54379159,-0.13207375 ,0.35799774])
    # camera_init_ori = np.array([-180*np.pi/180,0,90*np.pi/180])
    # client.aubo.movel_tf(pos=camera_init_pos,ori=camera_init_ori,frame_name='camera_center')  # 移动到相机初始位置
    # T_camera2base =client.aubo.tf_tree.get_transform('camera_center','world')
    # # run this only once
    # camera_init_pos, camera_init_ori = client.aubo.tf_tree.transform_to_pose(T_camera2base)
    # camera_init_ori = quaternion_to_rpy(np.array(camera_init_ori))
    # print("The init pose of camera",camera_init_pos,camera_init_ori)
    
    time.sleep(1)
    if client.robot_state != ROBOTSTATE['READY_TO_START']:
        raise ValueError('初始化失败，请检查连接')

    client.send_command('capture')
    response = client.receive_data()
    if not check_all(response):
        raise ValueError('无法获取全部物块的位置，请检查相机视野')
    cube0_pos_in_camera , cube0_ori_in_camera = client.get_cube_pose(response,0)  # x,y,theta
    cube1_pos_in_camera , cube1_ori_in_camera = client.get_cube_pose(response,1)
    cube2_pos_in_camera , cube2_ori_in_camera = client.get_cube_pose(response,2)\
    
    # modift the pos and ori to the pose in world
    # 将相机坐标旋转180°到工件坐标，cube0_ori_in_camera是相机坐标系下的物块实际位姿
    cube0_ori_in_camera[2] += np.pi
    cube1_ori_in_camera[2] += np.pi
    cube2_ori_in_camera[2] += np.pi
    cube0_pos_in_camera[2] = NORM_Z  # NORMZ是装配（物块1装配到物块0）的时候相机到物体0上表面的高度，这个是初始高度
    cube1_pos_in_camera[2] = NORM_Z
    cube0_ori_in_camera = rpy_to_quaternion(np.array(cube0_ori_in_camera))
    cube1_ori_in_camera = rpy_to_quaternion(np.array(cube1_ori_in_camera))
    cube2_ori_in_camera = rpy_to_quaternion(np.array(cube2_ori_in_camera))
    cube0_pos_in_camera , cube0_ori_in_camera = client.aubo.tf_tree.transform_pose(cube0_pos_in_camera,cube0_ori_in_camera,'camera_center','world')   # 把原来在A的点转换到B下
    cube1_pos_in_camera , cube1_ori_in_camera = client.aubo.tf_tree.transform_pose(cube1_pos_in_camera,cube1_ori_in_camera,'camera_center','world')   # 把原来在A的点转换到B下
    cube2_pos_in_camera , cube2_ori_in_camera = client.aubo.tf_tree.transform_pose(cube2_pos_in_camera,cube2_ori_in_camera,'camera_center','world')
    cube0_ori_in_camera = quaternion_to_rpy(np.array(cube0_ori_in_camera))
    cube1_ori_in_camera = quaternion_to_rpy(np.array(cube1_ori_in_camera))
    cube2_ori_in_camera = quaternion_to_rpy(np.array(cube2_ori_in_camera))
    
    # 通过以上的这些步骤，已经获得的就是世界坐标系下的物块位姿，但是还没给z赋值



    # 对物块2进行精定位
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
    client.aubo.movel_relative(cube2_pos_in_camera,cube2_ori_in_camera,"camera_center")
    time.sleep(1)#等待运动稳定
    cube2_pos , cube2_ori = client.aubo.get_pose('gripper_affine')
    cube2_ori = quaternion_to_rpy(np.array(cube2_ori))
    cube2_pos[2] += 0.0075 # 调整此值让夹爪抓到刚好接触到长圆形下表面



    # lzw
    # 对第一个 块0 执行硬定位
    if cube0_ori_in_camera[2]<-np.pi/2:
        offset = np.pi/2
    else:
        offset = -np.pi/2
    client.aubo.robot.set_end_max_line_acc(0.05)
    client.aubo.robot.set_end_max_line_velc(0.05) # 0.15
    # client.aubo.movel_tf(cube0_pos_in_camera,cube0_ori_in_camera,frame_name='gripper_center')  # 我称之为对齐函数，前两个是世界坐标，将gripper的坐标系对齐到世界坐标上
    time.sleep(1)
    safe_dist = np.array([0, 0, 0.05])  # 安全距离
    another_dist = np.array([0, 0, 0.005])
    cube0_pos_save = cube0_pos_in_camera + safe_dist  # 安全位置c
    client.aubo.movel_tf(cube0_pos_save,cube0_ori_in_camera,frame_name='gripper_center')  # 夹爪移动到物块0的安全位置
    time.sleep(1)
    client.aubo.robot.set_end_max_line_acc(0.05)
    client.aubo.robot.set_end_max_line_velc(0.05) # 0.15
    cube0_pos_grasp = cube0_pos_in_camera - another_dist # 抓取位置
    client.aubo.movel_tf(cube0_pos_grasp,cube0_ori_in_camera,frame_name='gripper_center')  # 夹爪移动到物块0的抓取位置
    client.gripper.close_gripper(speed=500, force=100) # 夹爪闭合
    time.sleep(1.5)
    client.gripper.open_gripper(speed=500) # 夹爪打开
    # client.aubo.robot.set_end_max_line_acc(0.05)
    # client.aubo.robot.set_end_max_line_velc(0.05) # 0.15
    client.aubo.movel_tf(cube0_pos_save,cube0_ori_in_camera,frame_name='gripper_center')  # 夹爪移动到物块0的安全位置
    time.sleep(1)
    client.aubo.robot.set_end_max_line_acc(0.05)
    client.aubo.robot.set_end_max_line_velc(0.20) # 0.15
    client.aubo.movel_relative(np.array([0,0,0]), np.array([0,0,offset]), frame_name='gripper_center')  # 旋转90度（90的正负需要判断）
    client.aubo.robot.set_end_max_line_acc(0.05)
    client.aubo.robot.set_end_max_line_velc(0.05) # 0.15
    time.sleep(1)

    client.aubo.movel_relative(safe_dist + another_dist, np.array([0,0,0]), frame_name='gripper_center')  # 夹爪移动到物块0的另一侧抓取位置
    client.gripper.close_gripper(speed=500, force=100) # 夹爪闭合
    time.sleep(1.5)
    client.gripper.open_gripper(speed=500) # 夹爪打开
    cube0_pos,cube0_ori = client.aubo.get_pose('gripper_center')
    client.aubo.tf_tree.add_node("home", "world", cube0_pos, cube0_ori)
    cube0_ori = quaternion_to_rpy(np.array(cube0_ori))
    client.aubo.movel_relative(- safe_dist - another_dist, np.array([0,0,0]), frame_name='gripper_center')  # 夹爪移动到物块0的安全位置

    # 对第二个 块1 执行硬定位
    if cube1_ori_in_camera[2]<-np.pi/2:
        offset = np.pi/2
    else:
        offset = -np.pi/2
    cube1_pos_save = cube1_pos_in_camera + safe_dist  # 安全位置c
    cube1_ori_another = cube1_ori_in_camera + np.array([0,0,offset]) # 另一侧的安全位置的位姿（直接再多转90，90的正负需要判断）
    # print("cube1_ori_in_camera:", cube1_ori_in_camera*180/np.pi)
    # print("cube1_ori_another:", cube1_ori_another*180/np.pi)
    client.aubo.movel_tf(cube1_pos_save,cube1_ori_another,frame_name='gripper_center', joint=True)  # 夹爪移动到物块1的另一侧安全位置
    client.aubo.movel_relative(safe_dist + another_dist, np.array([0,0,0]), frame_name='gripper_center')  # 夹爪移动到物块1的另一侧抓取位置
    client.gripper.close_gripper(speed=500, force=100) # 夹爪闭合
    time.sleep(1.5)
    client.gripper.open_gripper(speed=500) # 夹爪打开
    client.aubo.movel_relative(- safe_dist - another_dist, np.array([0,0,0]), frame_name='gripper_center')  # 夹爪移动到物块1的另一侧安全位置
    client.aubo.robot.set_end_max_line_acc(0.05)
    client.aubo.robot.set_end_max_line_velc(0.01) # 0.15
    # client.aubo.movel_relative(np.array([0,0,0]), np.array([0,0,-offset]), frame_name='gripper_center')  # 夹爪移动到物块1的安全位置（90的正负需要判断）
    client.aubo.movel_tf(cube1_pos_save,cube1_ori_in_camera,frame_name='gripper_center', joint=True)  # 夹爪移动到物块1的安全位置(正常的抓取角度)（90的正负需要判断）
    client.aubo.robot.set_end_max_line_acc(0.05)
    client.aubo.robot.set_end_max_line_velc(0.05) # 0.15
    client.aubo.movel_relative(safe_dist + another_dist, np.array([0,0,0]), frame_name='gripper_center')  # 夹爪移动到物块1的抓取位置
    client.gripper.close_gripper(speed=500, force=100) # 夹爪闭合
    time.sleep(1.5)
    # client.gripper.open_gripper(speed=500) # 夹爪打开
    client.aubo.movel_relative(- safe_dist - another_dist, np.array([0,0,0]), frame_name='gripper_center')  # 夹爪移动到物块1的安全位置

    # 把块1装入块0
    client.aubo.movel_tf(cube0_pos+safe_dist,cube0_ori,frame_name='gripper_center')  # 夹爪移动到物块0的上方
    client.aubo.robot.set_end_max_line_acc(0.05)
    client.aubo.robot.set_end_max_line_velc(0.05) # 0.15
    cube0_pos_insert = cube0_pos
    cube0_pos_insert[2] += 0.02  # 这个得调
    client.aubo.movel_tf(cube0_pos_insert, cube0_ori, frame_name='gripper_center') # 夹爪移动到装配位
    client.gripper.open_gripper(speed=500)  # 夹爪打开
    client.aubo.movel_tf(cube0_pos+safe_dist,cube0_ori,frame_name='gripper_center')  # 夹爪移动到物块0的上方






    # 抓取块2
    client.aubo.robot.set_end_max_line_acc(0.05)
    client.aubo.robot.set_end_max_line_velc(0.02) # 0.15
    client.aubo.movel_tf(cube2_pos+safe_dist,cube2_ori,'gripper_center')
    client.aubo.movel_tf(cube2_pos,cube2_ori,'gripper_center')
    # time.sleep(1)
    client.gripper.close_gripper(speed=500, force=100)
    time.sleep(0.8)
    client.aubo.movel_relative(-safe_dist,np.array([0,0,0]),'gripper_center')

    # 把块2撞到块1上
    Z_cube2_ground = 34.7  # 全是理论值，理论上无需调整
    Z_gripper2_ground = 10  # 夹爪在末端接触地面时，夹爪中心到地面的高度
    pos_slot2home = np.array([-0.01, 0.0, -(Z_cube2_ground - Z_gripper2_ground) / 1000])
    ori_slot2home = np.array([-15 * np.pi / 180, 0, 0])
    ori_slot2home = rpy_to_quaternion(ori_slot2home)
    slot_offset = np.array([0.0, 0.0, 0.009])# 和抓取的深度有关，对应不同的抓取位置需要调整

    client.aubo.tf_tree.add_node("slot", "home", pos_slot2home, ori_slot2home)
    client.aubo.tf_tree.add_node("slot_offset", "slot", -slot_offset, np.array([0, 0, 0, 1]))
    client.aubo.tf_tree.add_node("safe_slot", "slot", -safe_dist, np.array([0, 0, 0, 1]))
    # T_slot2world = client.aubo.tf_tree.get_transform("slot_offset", "world")
    # slot_pos, slot_ori = client.aubo.tf_tree.transform_to_pose(T_slot2world)
    slot_pos,slot_ori = client.aubo.get_pose('slot_offset','world')
    slot_ori = quaternion_to_rpy(np.array(slot_ori))
    # T_safe_slot2world = client.aubo.tf_tree.get_transform("safe_slot", "world")
    # safe_slot_pos, safe_slot_ori = client.aubo.tf_tree.transform_to_pose(T_safe_slot2world)
    safe_slot_pos,safe_slot_ori = client.aubo.get_pose('safe_slot','world')
    safe_slot_ori = quaternion_to_rpy(np.array(safe_slot_ori))
    client.aubo.tf_tree.print_tree_structure()
    client.aubo.movel_tf(safe_slot_pos, safe_slot_ori, 'gripper_center')
    # time.sleep(1)
    client.aubo.robot.set_end_max_line_acc(0.08)
    client.aubo.robot.set_end_max_line_velc(0.08)
    client.aubo.movel_tf(slot_pos, slot_ori, 'gripper_center')
    # time.sleep(1)
    client.gripper.open_gripper(speed=500)
    time.sleep(0.5)
    client.aubo.robot.set_end_max_line_acc(0.1)
    client.aubo.robot.set_end_max_line_velc(0.15)
    client.aubo.movel_tf(pos=init_pos, ori=init_ori, frame_name='flange_center')




    client.disconnect()
    client.aubo.disconnect()


if __name__ == '__main__':
    main()