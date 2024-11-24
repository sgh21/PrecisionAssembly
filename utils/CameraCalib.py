# client.py
import socket
import pickle
import numpy as np
import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)
sys.path.append(workspace)
from utils.config import NORM_Z ,CAMERA_CONFIG




def main():
 
   
    # 调整测量获得的图片参数
    box_points = np.array([
        [1317,842],
        [1752,841],
        [1754,1275]
        ])

    cube_size = [0.05,0.05]

    width_pixel = np.linalg.norm(box_points[0]-box_points[1])
    height_pixel = np.linalg.norm(box_points[1]-box_points[2])

    fx = NORM_Z*width_pixel / cube_size[0]
    fy = NORM_Z*height_pixel / cube_size[1]

    # # 读取 YAML 文件
    # import yaml
    # with open(CAMERA_CONFIG, 'r') as file:
    #     camera_config = yaml.safe_load(file)
   
    

    print(f"Please updated fx: {fx}, fy: {fy} in camera_config.yaml")
if __name__ == '__main__':
    main()