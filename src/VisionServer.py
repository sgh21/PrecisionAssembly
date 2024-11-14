# server.py
import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)
sys.path.append(workspace)
import socket
import pickle
import cv2
import numpy as np
import threading
from ultralytics import YOLO
from src.MVScontrol import MVSInit, capture_frame
from utils.config import YOLO_MODEL_PATH, COLOR_DICT,CAMERA_CONFIG,NORM_Z,YOLO_MODEL_PATH_CIRCLE
from utils.VisionUtils import *

class VisionObject:
    def __init__(self,image = None): 
        self.image = image
        self.xyxy = None
        self.roi_img = None
        self.roi_mask = None
        self.cls = None
        self.cv_pose = None

class VisionServer:
    def __init__(self,host='', port=2024):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.img_count = 0
        print('Server start ,waiting for connection...')

    def init_camera(self,camera_config = CAMERA_CONFIG):
        self.cam, self.data_buf, self.nPayloadSize = MVSInit()
        try :
            import yaml 
            file_path = os.path.join(workspace,camera_config)
            with open(file_path,"r") as f:
                camera_config = yaml.safe_load(f)
                self.intrinsics = np.array(camera_config["intrinsic"])
                self.distortion = np.array(camera_config["distortion_coefficients"])
        except Exception as e:
            print(e)
            self.intrinsics = None
            self.distortion = None
            return False
        
        return True

    def load_yolo(self,weights = YOLO_MODEL_PATH):
        try:
            weight_file = os.path.join(workspace, weights)
            self.model = YOLO(weight_file)
        except Exception as e:
            print(e)
            return False
        return True
    
    def load_yolo_circle(self,weights = YOLO_MODEL_PATH_CIRCLE):
        try:
            weight_file = os.path.join(workspace, weights)
            self.model_circle = YOLO(weight_file)
        except Exception as e:
            print(e)
            return False
        return True
    
    def process_image_yolo(self,image,padding = 50,debug = False):
        '''
        使用YOLO模型进行目标检测
        input: image: np.ndarray, 图像数据
        output: reslut_list<VisionObjects>: list, 检测结果列表
        '''
        
        # 模型推理
        t1 = cv2.getTickCount()
        result = self.model(image)[0]
        t2 = cv2.getTickCount()
        print(f'YOLO推理时间：{(t2 - t1) / cv2.getTickFrequency()}s')
        # 初始化结果图像
        img_result = image.copy()
        h , w = image.shape[:2]
        # 处理检测结果
        boxes = result.boxes  # 边界框
        # masks = result.masks  # 掩码

        result_objs = [] 
        for i in range(len(boxes)):
            result_obj = VisionObject()
            print(f'检测到目标：{boxes.cls[i]}')
            # 获取类别和颜色
            cls = int(boxes.cls[i])
            color = COLOR_DICT.get(cls, (255, 255, 255))

            # 获取边界框坐标并转换为整数
            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
            x1, y1 = max(x1-padding, 0), max(y1-padding, 0)
            x2, y2 = min(x2+padding, w-1), min(y2+padding, h-1)
            
            # 提取感兴趣区域（ROI）
            roi_img =  np.copy(image[y1:y2, x1:x2])
            if debug:
                cv2.rectangle(img_result, (x1, y1), (x2, y2), color, 2)
            result_obj.image = img_result
            result_obj.xyxy = np.array([x1, y1, x2, y2])
            result_obj.roi_img = roi_img
            result_obj.cls = cls
            result_objs.append(result_obj)

            
        return result_objs
    def is_in_range(self, boxes, boundingbox, num=2):
        """
        判断有多少个 boxes 的中心在 boundingbox 内
        参数：
        - boxes: 检测到的边界框列表
        - boundingbox: 目标边界框，格式为 [x1, y1, x2, y2]
        - num: 判断的阈值，默认值为 2
        返回：
        - bool: 如果在 boundingbox 内的 boxes 数量等于 num，返回 True，否则返回 False
        """
        count = 0
        center = np.zeros(2)
        x1_b, y1_b, x2_b, y2_b = boundingbox
        centers = []
        for i in range(len(boxes)):
            # 获取边界框坐标并转换为整数
            center = np.zeros(2)
            x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
            center[0] = (x1 + x2) / 2
            center[1] = (y1 + y2) / 2

            # 判断中心点是否在 boundingbox 内
            if x1_b <= center[0] <= x2_b and y1_b <= center[1] <= y2_b:
                count += 1
                centers.append(center)
        # 没有找到足够的圆心        
        if count != num:
            return None
        # 圆心连线垂直于水平基准，没有问题
        is_vertical = np.abs((centers[0][0]-centers[1][0])\
                            /(centers[0][1]-centers[1][0]))<= 0.3
        if is_vertical:
            return 0
        # 圆心连线基本和水平轴平行，需要旋转±90 如果圆心连线在上，则正转，否则反转
        is_upper = (centers[0][1]+centers[1][1])/2 >= (y1_b + y2_b)/2
        if is_upper:
            return -1
        
        return 1
    
    def find_circle(self,img,result_objs,cls):
        circle_found = False
        boundingbox = np.zeros(4)
        while not circle_found:
            result = self.model_circle(img)[0]     
            boxes = result.boxes
            obj:VisionObject = [obj for obj in result_objs if obj.cls == cls][0]
            boundingbox = obj.xyxy 
            modify_angle = self.is_in_range(boxes,boundingbox,num=2)
            if  modify_angle is not None:
                circle_found = True
                return modify_angle*np.pi/2
            img = capture_frame(self.cam,self.data_buf,self.nPayloadSize)
        return None
    
    def PCA(self,contours,image_size=(3072,2048),debug = False,img = None):
        # 对于正方形等特殊情况，主方向并不明确，慎用
        # 1. 创建与图像大小相同的空白掩码
        mask = np.zeros(image_size, dtype=np.uint8)

        # 2. 在掩码上绘制填充的轮廓
        cv2.drawContours(mask, [contours], -1, color=255, thickness=cv2.FILLED)

        # 3. 提取轮廓内部的所有点坐标
        points = cv2.findNonZero(mask)  # 返回 [[x1, y1], [x2, y2], ...]
        points = points.reshape(-1, 2)  # 转换为 N x 2 的数组

        # 4. 计算质心（平均位置）
        mean = np.mean(points, axis=0)

        # 5. 去中心化
        data_pts = points - mean

        # 6. 计算协方差矩阵
        cov_matrix = np.cov(data_pts, rowvar=False)

        # 7. 进行特征值分解，获取特征值和特征向量
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

        # 8. 获取主轴方向（对应最大特征值的特征向量）
        largest_idx = np.argmax(eigenvalues)
        principal_axis = eigenvectors[:, largest_idx]

        # 9. 计算主轴与水平轴的夹角
        angle = np.arctan2(principal_axis[1], principal_axis[0])
        # 归一化到 [-pi/2, pi/2]
        angle = (angle + np.pi) % np.pi
        angle = angle - np.pi  if angle > np.pi/2 else angle
        # 再归一化到 [-pi/4, pi/4]
        if angle < -np.pi/4:
            angle = angle + np.pi/2
        elif angle > np.pi/4:
            angle = angle - np.pi/2

        angle_deg = np.degrees(angle)
        print(f"主轴与水平轴的夹角为：{angle_deg:.2f} 度")
        # 在原图上绘制主轴
        if debug:
            if img is None:
                raise ValueError("img is None")
            cv2.circle(img, tuple(np.round(mean).astype(int)), 5, (0, 255, 0), -1)
            cv2.line(img, tuple(np.round(mean).astype(int)), tuple(np.round(mean + principal_axis * 100).astype(int),), (0, 0, 255), 2)
            cv2.namedWindow('PCA', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('PCA',1536,1024)
            cv2.imshow('PCA', img)
            if cv2.waitKey(50) & 0xFF == ord('q'):
                cv2.destroyWindow('PCA')
        return angle
    
    def process_image_opencv(self,image,result_objs,just_detect = False,area_threshold = 0,debug = False):
        '''
        使用OpenCV进行图像处理
        input: 
                image: np.ndarray, 原始图像数据
                result_objs<VisionObjects>: list, yolo处理结果列表
        output:
                pose_list<x,y,theta>: list, 位姿估计结果列表
        '''
        
        h , w = image.shape[:2]
        pose_list = []
        if not just_detect:
            count = 0
            for obj in result_objs:
                x1, y1, x2, y2 = obj.xyxy
                cls = obj.cls
                roi_img = obj.roi_img
                # roi_mask = obj.roi_mask
                # 对ROI图像进行滤波处理
                # roi_img = fliter(roi_img)# 感觉图像比较干净，没大有用
                # 对ROI图像进行灰度增强
                gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
                enhance_gray = enhance_contrast(gray,show=False)
                # 噪声也会增强，因此要滤波
                enhance_gray = fliter(enhance_gray)
                # cv2.imwrite(f'{workspace}/dataset/test_images/enhance_gray.jpg',enhance_gray)
                # 使用canny检测提取边缘 阈值是梯度变化，背景是0，所以阈值变化应该很大，这里设置为150，250
                edges = cv2.Canny(cv2.bitwise_not(enhance_gray), 150, 250)
                max_contours,contours = find_contours(edges,threshold=area_threshold)
                while len(contours) == 0:
                    # 闭运算，确保边缘连续
                    kernel = np.ones((2,2),np.uint8)
                    edges = cv2.dilate(edges,kernel,iterations = 2)
                    edges = cv2.erode(edges,kernel,iterations = 1)
                    max_contours,contours = find_contours(edges,threshold=area_threshold)
                #  轮廓精细化
                contour_points = max_contours.reshape(-1, 1, 2).astype(np.float32)
                # 定义精细化参数
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
                # 精细化轮廓点
                refined_contour = cv2.cornerSubPix(enhance_gray, contour_points, (5, 5), (-1, -1), criteria)


                # 最小外接矩形
                # rect = cv2.minAreaRect(max_contours)
                rect = cv2.minAreaRect(refined_contour)
                
                # 绘制形心和主方向
                center, angle,center_rect = cal_rect_angle(rect,refined_contour)
        
               
                obj.cv_pose = np.array([center[0]+x1,center[1]+y1,angle])
                print(f"Pose of cube{cls}:",obj.cv_pose)
                if debug:
                    # 清空文件夹
                    os.system(f'rm -rf {workspace}/dataset/test_images/*')
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    width , height = self.cal_rect_length(box+center)
                    print(f"The width and length of {cls}:width:{width},height:{height}",)
                    # cv2.imshow('enhance_gray',enhance_gray)
                    # cv2.imshow('edges',edges)
                    cv2.imwrite(f'{workspace}/dataset/test_images/edges{cls}_{self.img_count}_{count}.jpg',edges)
                    cv2.imwrite(f'{workspace}/dataset/test_images/enhance_gray{cls}_{self.img_count}_{count}.jpg',enhance_gray)
                    cv2.drawContours(roi_img, [box], 0, (0, 255, 0), 2)
                    # 绘制边缘到原图
                    cv2.drawContours(roi_img, contours, -1, (0, 0, 255), 2)
                   
                    direction = (np.cos(np.radians(angle)), np.sin(np.radians(angle)))
                    cv2.circle(roi_img, center, 5, (255, 0, 0), -1)
                    if direction != (0, 0):
                        end_point = (int(center[0] + direction[0] * 50), int(center[1] + direction[1] * 50))
                        cv2.line(roi_img, center, end_point, (255, 0, 0), 2)
                    cv2.circle(roi_img, center_rect, 5, (255, 255, 0), -1)
                    if direction != (0, 0):
                        end_point = (int(center_rect[0] + direction[0] * 50), int(center_rect[1] + direction[1] * 50))
                        cv2.line(roi_img, center_rect, end_point, (255, 255, 0), 2)
                    cv2.imshow('result', roi_img)
                    cv2.imwrite(f'{workspace}/dataset/test_images/result{cls}_{self.img_count}_{count}.jpg',roi_img)
                    count += 1
                    # if cv2.waitKey(500)&0xFF == ord('q'):
                    #     cv2.destroyAllWindows()
                    #     break
                    # cv2.waitKey(0) 
        # 根据检测数据进行位姿估计
        for obj in result_objs:
            x1, y1, x2, y2 = obj.xyxy
            cls = obj.cls
            if obj.cv_pose is not None:
                center = obj.cv_pose[:2]
                theta = obj.cv_pose[2]
            else:
                center = ((x1 + x2) // 2, (y1 + y2) // 2)
                theta = 0
            pose_in_picture = np.array([center[0], center[1], 1])
            if self.intrinsics is not None:
                # instrinsics = self.intrinsics
                instrinsics = cv2.getOptimalNewCameraMatrix(self.intrinsics, self.distortion, (w, h), 1, (w, h))[0]
                # 计算相机坐标系下的坐标
                pose_in_camera = np.linalg.inv(instrinsics) @ pose_in_picture
                # 计算世界坐标系下的坐标
                pose_in_camera = pose_in_camera * NORM_Z#要获得比较准确的物块在相机坐标系下的Z
                pose = np.ones(3)
                pose[:2] = pose_in_camera[:2]
                pose[2] = theta

            pose = np.append(pose,cls)
            pose_list.append(pose)
        self.img_count += 1
        return pose_list
    
    def cal_rect_length(self,box):
        w = 3072
        h = 2048
        point1 = np.array([box[0][0],box[0][1],1])
        point2 = np.array([box[1][0],box[1][1],1])
        point3 = np.array([box[2][0],box[2][1],1])

        if self.intrinsics is not None:
                intrinsics = cv2.getOptimalNewCameraMatrix(self.intrinsics, self.distortion, (w, h), 1, (w, h))[0]
                # 计算相机坐标系下的坐标
                pose1_in_camera = np.linalg.inv(intrinsics) @ point1
                pose2_in_camera = np.linalg.inv(intrinsics) @ point2
                pose3_in_camera = np.linalg.inv(intrinsics) @ point3
                # 计算世界坐标系下的坐标
                pose1_in_camera = pose1_in_camera * NORM_Z#要获得比较准确的物块在相机坐标系下的Z
                pose2_in_camera = pose2_in_camera * NORM_Z
                pose3_in_camera = pose3_in_camera * NORM_Z

                width = np.linalg.norm(pose1_in_camera-pose2_in_camera)
                length = np.linalg.norm(pose2_in_camera-pose3_in_camera)
        return width ,length
    
    def warmup(self):
        print("Start warming up...")
        while not self._stop_warmup:
            # 拍照
            img = capture_frame(self.cam,self.data_buf,self.nPayloadSize)
            # 推理
             # 模型推理
            t1 = cv2.getTickCount()
            result = self.model(img)[0]
            t2 = cv2.getTickCount()
            print(f'YOLO推理时间：{(t2 - t1) / cv2.getTickFrequency()}s')
            # 初始化结果图像
            img_result = img.copy()
            h , w = img.shape[:2]
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
            
                cv2.namedWindow('Warmup', cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Warmup',1536,1024)
                cv2.rectangle(img_result, (x1, y1), (x2, y2), color, 2)
                cv2.imshow('Warmup',img_result)
                if cv2.waitKey(50) & 0xFF == ord('q'):
                    cv2.destroyWindow('Warmup')
                    continue
        print("Stop warming up")
          

    def start_warmup(self):
        self._stop_warmup = False
        self.warmup_thread = threading.Thread(target=self.warmup)
        self.warmup_thread.start()

    def stop_warmup(self):
        self._stop_warmup = True
        self.warmup_thread.join()
        
def main():
    host = '0.0.0.0'
    port = 2024  # 端口号，可以根据需要修改

    visionserver = VisionServer(host,port)
    if(not visionserver.init_camera()):
        raise Exception("Camera init failed")
    if(not visionserver.load_yolo(YOLO_MODEL_PATH)):
        raise Exception("YOLO model load failed")
    
    if(not visionserver.load_yolo_circle(YOLO_MODEL_PATH_CIRCLE)):
        raise Exception("YOLO_CIRCLE model load failed")
    
    # 等待客户端连接,阻塞，开始热身
    visionserver.start_warmup()
    conn , addr = visionserver.server_socket.accept()
    visionserver.stop_warmup()
    while True:
        # 接收客户端的请求
        data = conn.recv(1024)
        if not data:
            continue
        command = data.decode()
        print(f'收到指令：{command}')

        if command == 'capture':
            t1 = cv2.getTickCount()
            # 进行图像处理
            # result = capture_frame(cam,data_buf,nPayloadSize,'test',targrt_dir=save_dir,show = False)
            # image = cv2.imread(f'{workspace}/dataset/test_images/capture0_0_1.jpg')
            image = capture_frame(visionserver.cam,visionserver.data_buf,visionserver.nPayloadSize)
            t3 = cv2.getTickCount()
            # 处理图像
            yolo_result = visionserver.process_image_yolo(image,debug=False)
            t4 = cv2.getTickCount()
            result = visionserver.process_image_opencv(\
                                    image,yolo_result, \
                                    just_detect = False,\
                                    debug= True,\
                                    area_threshold=8000)
            t5 = cv2.getTickCount()
            # 序列化数
            data_to_send = pickle.dumps(result)
            # 发送数据长度
            conn.sendall(len(data_to_send).to_bytes(4, byteorder='big'))
            # 发送数据
            conn.sendall(data_to_send)
            print('处理结果已发送给客户端')
            t2 = cv2.getTickCount()
            print(f'Capture处理时间：{(t3 - t1) / cv2.getTickFrequency()}s')
            print(f'YOLO处理时间：{(t4 - t3) / cv2.getTickFrequency()}s')
            print(f'OpenCV处理时间：{(t5 - t4) / cv2.getTickFrequency()}s')
            print(f'数据传输时间：{(t2 - t5) / cv2.getTickFrequency()}s')

        elif command == 'modify_angle0':
            image = capture_frame(visionserver.cam,visionserver.data_buf,visionserver.nPayloadSize)
            yolo_result = visionserver.process_image_yolo(image,debug=False)
            modify_angle = visionserver.find_circle(image,yolo_result,0)
            if modify_angle is not None:
                # 序列化数
                data_to_send = pickle.dumps(modify_angle)
                # 发送数据长度
                conn.sendall(len(data_to_send).to_bytes(4, byteorder='big'))
                # 发送数据
                conn.sendall(data_to_send)
                print('处理结果已发送给客户端')
                print('The modify angle should be:',modify_angle*180/np.pi)
            else:
                raise ValueError('Can not find the right modify angle')
        elif command == 'modify_angle1':
            image = capture_frame(visionserver.cam,visionserver.data_buf,visionserver.nPayloadSize)
            yolo_result = visionserver.process_image_yolo(image,debug=False)
            modify_angle = visionserver.find_circle(image,yolo_result,1)
            if modify_angle is not None:
                # 序列化数
                data_to_send = pickle.dumps(modify_angle)
                # 发送数据长度
                conn.sendall(len(data_to_send).to_bytes(4, byteorder='big'))
                # 发送数据
                conn.sendall(data_to_send)
                print('处理结果已发送给客户端')
                print('The modify angle should be:',modify_angle*180/np.pi)
            else:
                raise ValueError('Can not find the right modify angle')
        elif command == 'exit':
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    conn.close()
    visionserver.server_socket.close()

if __name__ == '__main__':
    main()