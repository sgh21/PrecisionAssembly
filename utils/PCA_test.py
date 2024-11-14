import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)
sys.path.append(workspace)

import cv2

from src.VisionServer import VisionServer
from src.MVScontrol import capture_frame
from config import YOLO_MODEL_PATH

host = '0.0.0.0'
port = 2024  # 端口号，可以根据需要修改

visionserver = VisionServer(host,port)
if(not visionserver.init_camera()):
    raise Exception("Camera init failed")
if(not visionserver.load_yolo(YOLO_MODEL_PATH)):
    raise Exception("YOLO model load failed")


while True:
    image = capture_frame(visionserver.cam,visionserver.data_buf,visionserver.nPayloadSize)
    # 处理图像
    yolo_result = visionserver.process_image_yolo(image,debug=False)
    result = visionserver.process_image_opencv(\
                            image,yolo_result, \
                            just_detect = False,\
                            debug= False,\
                            area_threshold=2000)