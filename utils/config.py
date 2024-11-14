# config.py
import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)
sys.path.append(workspace)
# 模型路径
YOLO_MODEL_PATH = f'{workspace}/weights/yolov8-seg.pt'  # 替换为您的YOLOv8-Seg模型路径
YOLO_MODEL_PATH_CIRCLE = f'{workspace}/weights/yolov8-seg-for-circle.pt'
CAMERA_CONFIG = f'{workspace}/config/camera_config.yaml'  # 相机配置文件路径
template_dir = os.path.join(workspace, 'dataset/template')
# 模板图像路径（根据类别索引）
TEMPLATE_PATHS = {
    0: f'{template_dir}/capture_template0.jpg',       # 类别0：正方体模板路径
    1: f'{template_dir}/capture_template1.jpg',      # 类别1：楔形模板路径
    2: f'{template_dir}/capture_template2.jpg'    # 类别2：长圆形模板路径
}

# 类别颜色字典（用于结果显示）
COLOR_DICT = {
    0: (0, 255, 0),    # 类别0：正方体，绿色
    1: (255, 0, 0),    # 类别1：楔形，蓝色
    2: (0, 0, 255),     # 类别2：长圆形，红色
    3: (0,255,255),
    4: (255,255,0)
}
NORM_Z = 0.40015# 归一化高度 到标定板时0.39169 0.415803 0.403103 0.37874 0.4125701+0.00358 0.40015
# ORB特征检测器参数
ORB_FEATURES = 5000  # 特征点数量

# 匹配点数量阈值
MIN_MATCH_COUNT = 10

# 其他参数
THRESHOLD_VALUE = 127  # 二值化阈值
