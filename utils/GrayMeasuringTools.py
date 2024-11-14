# Function: This script is used to measure the grayscale value of a specific range 
# in an image and real-time show the mask and result of the grayscale range 
# Author: Shen Guang-hui
# Create date: 2024-08-15
# Version: 1.0
# Environment: Windows 11, Python 3.10.0
import os 
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace = os.path.dirname(current_dir)
sys.path.append(workspace)
import cv2  
import numpy as np  

path = f'{workspace}/dataset/test_images/enhance_gray0_0_0.jpg'  # 图像的位置，使用时仅需修改这一属性

# 滑动条的回调函数，获取滑动条位置处的值  
def empty(a):  
    gray_min = cv2.getTrackbarPos("Gray Min", "TrackBars")  
    gray_max = cv2.getTrackbarPos("Gray Max", "TrackBars")  
    print(gray_min, gray_max)  
    return gray_min, gray_max  
    
# 创建一个窗口，放置2个滑动条  
cv2.namedWindow("TrackBars")  
cv2.resizeWindow("TrackBars", 640, 100)  
cv2.createTrackbar("Gray Min", "TrackBars", 0, 255, empty)  
cv2.createTrackbar("Gray Max", "TrackBars", 255, 255, empty)  
  
while True:  
    img = cv2.imread(path)
    if img is None:
        print(f"无法读取图像文件：{path}")
        break

    # 将图像转换为灰度图像
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  
    
    # 调用回调函数，获取滑动条的值  
    gray_min, gray_max = empty(0)  
    lower = np.array([gray_min])  
    upper = np.array([gray_max])  
    
    # 获得指定灰度范围内的掩码  
    mask = cv2.inRange(gray_img, lower, upper)  
    
    # 对原图图像进行按位与的操作，掩码区域保留  
    imgResult = cv2.bitwise_and(img, img, mask=mask)  
    
    # cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    # cv2.namedWindow("Result", cv2.WINDOW_NORMAL) 
    # cv2.resizeWindow("Mask", 1536, 1024) 
    # cv2.resizeWindow("Result", 1536, 1024) 
    cv2.imshow("Mask", mask)  
    cv2.imshow("Result", imgResult)  
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()