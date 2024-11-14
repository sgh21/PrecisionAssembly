import os 
import sys
import cv2
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)
sys.path.append(workspace)
from src.MVScontrol import *
from utils.config import *
from ultralytics import YOLO

def yolo_predict(model,image,show = True):
     # 模型推理
    t1 = cv2.getTickCount()
    result = model(image)[0]
    t2 = cv2.getTickCount()
    print(f'YOLO推理时间：{(t2 - t1) / cv2.getTickFrequency()}s')
    # 初始化结果图像
    img_result = image.copy()
    h , w = image.shape[:2]
    # 处理检测结果
    boxes = result.boxes  # 边界框
    for i in range(len(boxes)):
        print(f'检测到目标：{boxes.cls[i]}')
        # 获取类别和颜色
        cls = int(boxes.cls[i])
        color = COLOR_DICT.get(cls, (255, 255, 255))

        # 获取边界框坐标并转换为整数
        x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
        x1, y1 = max(x1, 0), max(y1, 0)
        x2, y2 = min(x2, w-1), min(y2, h-1)
    
        if show:
            cv2.namedWindow('dst', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('dst',1536,1024)
            cv2.rectangle(img_result, (x1, y1), (x2, y2), color, 2)
            cv2.imshow('dst',img_result)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                cv2.destroyWindow('dst')
                return

def main(weights):
    # 初始化相机
    cam, data_buf, nPayloadSize = MVSInit()
    try:
        # 加载YOLO模型
        weight_file = os.path.join(workspace, weights)
        model = YOLO(weight_file)
    except Exception as e:
        print(e)
    while 0xFF != ord('q'):
        # 获取图片
        img = capture_frame(cam,data_buf,nPayloadSize)
        # 预测并显示结果
        yolo_predict(model,img,show=True)


if __name__ == "__main__":
    weights = f"{workspace}/weights/yolov8-seg-for-circle.pt"
    main(weights)
    