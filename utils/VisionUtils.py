import cv2
import numpy as np
# lower_bound 和 upper_bound 是用于提取前景的阈值，最好不要太低，防止丢失关键信息，调整时还需要联动tileGridSize保证局部不过曝
def enhance_contrast(img,lower_bound=0,upper_bound= 105,clipLimit = 2.0,tileGridSize =(64,64),show = False):
        gray = img.copy()
        if len(gray.shape) != 2:
            raise ValueError("输入图像必须是灰度图像")
        # 创建一个掩码
        mask = cv2.inRange(gray, lower_bound, upper_bound)
        # 提取掩码范围内的部分
        extracted = cv2.bitwise_and(gray, gray, mask=mask)
        # 对提取的部分进行均衡化
        # equalized = cv2.equalizeHist(extracted)
        clahe = cv2.createCLAHE(clipLimit=clipLimit, tileGridSize=tileGridSize)
        equalized = clahe.apply(extracted)
        # equalized = extracted.copy()
        # 将均衡化后的部分合并回原图
        enhanced_gray = gray.copy()
        enhanced_gray[mask > 0] = equalized[mask > 0]
        enhanced_gray[mask<=0] = 255

        # mask = cv2.inRange(enhanced_gray, lower_bound, upper_bound+100)
        # extracted = cv2.bitwise_and(enhanced_gray, enhanced_gray, mask=mask)
        # # 对提取的部分进行均衡化
        # enhanced_gray[mask > 0] = extracted[mask > 0]
        # enhanced_gray[mask<=0] = 255
        if show:
            import matplotlib.pyplot as plt
            # 绘制原始灰度图和均衡化后的灰度图
            plt.figure(figsize=(12, 6))
            plt.subplot(1, 2, 1)
            plt.title("Original Grayscale Image")
            plt.imshow(gray, cmap='gray')
            plt.axis('off')
            
            plt.subplot(1, 2, 2)
            plt.title("Enhanced Grayscale Image")
            plt.imshow(enhanced_gray, cmap='gray')
            plt.axis('off')
            
            plt.show()
        return enhanced_gray

def fliter(image,d=9,sigmaColor=75,sigmaSpace=75):
    
    blur = cv2.bilateralFilter(image, d, sigmaColor, sigmaSpace)
    return blur
def preprocess_mask(mask):
    """
    对掩码进行预处理，二值化并去噪
    """
    from utils.config import THRESHOLD_VALUE
    _, binary_mask = cv2.threshold(mask, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY_INV)
    # 可以添加其他预处理步骤，例如形态学操作
    return binary_mask

def segment_hsv(img, lower_hsv, upper_hsv):
        # 将图像从 BGR 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 创建掩码
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        
        # 使用掩码提取目标区域
        segmented_img = cv2.bitwise_and(img, img, mask=mask)
        
        return segmented_img, mask
def find_contours(edge,threshold=2000):
    """
    从掩码中提取轮廓
    """
    contours, _ = cv2.findContours(edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # 筛选掉面积小于阈值的轮廓
    filter_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= threshold]
    # 找到最大面积的轮廓
    if len(filter_contours) == 0:
        return None,filter_contours
    max_contour = max(filter_contours, key=cv2.contourArea)
   
    return max_contour,filter_contours 


def cal_rect_angle(rect,contour):
    # 获取最小外接矩形的参数
    center = rect[0]
    size = rect[1]
    angle = rect[2]
    width, height = size
    # 将中心坐标转换为整数
    center = (int(center[0]), int(center[1]))
    M = cv2.moments(contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else :
        cx,cy = center
    if 0 <= angle <= 90:# 合法角度
        # 判断长宽比
        aspect_ratio = width / height
        if 0.8 <= aspect_ratio <= 1.2:
            # 长宽长度相近，以相机 x 轴为原点，角度归一化到 [-45, 45]
            angle = angle - 90 if angle > 45 else angle
        else :
            # 长宽长度不相近，比较长宽，定义角度为短边和 x 轴的夹角
            angle = angle - 90 if width > height else angle

        return (cx,cy), angle,center
    else :
        print("The original angle is:",angle)
        raise ValueError("角度不合法")
    

def edge_drawing_circle_detection(image_path):
    # 读取图像
    img = cv2.imread(image_path)
    if img is None:
        print("无法读取图像")
        return
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 创建 Edge Drawing 对象
    edge_drawer = cv2.ximgproc.createEdgeDrawing()

    edge_params = cv2.ximgproc.EdgeDrawing.Params()
    # # 设置 Edge Drawing 参数（根据需要进行调整）
    # edge_drawer.params.EdgeDetectionOperator = cv2.ximgproc.EDOPERATORS.SOBEL
    # edge_drawer.params.ThinningType = cv2.ximgproc.ED_THINNING.ZHANGSUEN
    # edge_drawer.params.MinPathLength = 10  # 最小边缘长度
    # edge_drawer.params.MaxDeviationAngle = 15  # 最大偏差角度

    # 运行 Edge Drawing 算法
    edge_drawer.detectEdges(gray)

    # 检测椭圆（包括圆）
    edge_drawer.detectEllipses()

    # 获取检测到的椭圆
    ellipses = edge_drawer.getEllipses()

    # 绘制结果
    for ellipse in ellipses:
        # 获取椭圆参数
        center = (int(ellipse.center.x), int(ellipse.center.y))
        axes = (int(ellipse.axes.width / 2), int(ellipse.axes.height / 2))
        angle = ellipse.angle

        # 判断是否为圆（长短轴接近）
        if abs(axes[0] - axes[1]) <= 5:  # 您可以根据需要调整阈值
            # 绘制圆
            cv2.ellipse(img, center, axes, angle, 0, 360, (0, 255, 0), 2)
            cv2.circle(img, center, 2, (0, 0, 255), -1)

    # 显示结果
    cv2.imshow("Circle Detection", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    import os 
    import sys
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace = os.path.dirname(current_dir)
    sys.path.append(workspace)
    import cv2  
    import numpy as np  

    path = f'{workspace}/dataset/test_images/enhance_gray0_0_0.jpg' 
    edge_drawing_circle_detection("./test_data/1.jpg")