import cv2
import numpy as np
import os

def detect_and_compute(image, detector):
    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 检测关键点并计算描述符
    keypoints, descriptors = detector.detectAndCompute(gray, None)
    return keypoints, descriptors

def compute_centroid(keypoints):
    # 计算关键点的重心
    if not keypoints:
        return None
    x_coords = [kp.pt[0] for kp in keypoints]
    y_coords = [kp.pt[1] for kp in keypoints]
    centroid = (np.mean(x_coords), np.mean(y_coords))
    return centroid

def main():
    theta = -5  # 旋转角度增量，可根据需要修改
    num_images = 10  # 图像数量

    dir_path = './calib_data/rotation'  # 指定图片所在的目录路径
    if not os.path.exists(dir_path):
        print(f"目录 {dir_path} 不存在，请检查路径")
        return

    # 读取基准图像
    base_image_path = os.path.join(dir_path, "1.png")
    base_image = cv2.imread(base_image_path)
    if base_image is None:
        print(f"无法找到图像 {base_image_path}")
        return
    height, width = base_image.shape[:2]
    size = (width, height)

    # 初始化 ORB 特征检测器
    orb = cv2.ORB_create()

    # 检测基准图像的特征
    kp_base, des_base = detect_and_compute(base_image, orb)
    # 计算基准图像特征点的重心
    centroid_base = compute_centroid(kp_base)
    if centroid_base is None:
        centroid_base = (width // 2, height // 2)

    accumulated_image = base_image.astype(np.float32)  # 初始化累加图像

    for i in range(2, num_images + 1):
        img_filename = f"{i}.png"
        img_path = os.path.join(dir_path, img_filename)
        image = cv2.imread(img_path)
        if image is None:
            print(f"无法找到图像 {img_path}")
            continue

        # 检测当前图像的特征
        kp_curr, des_curr = detect_and_compute(image, orb)
        # 计算当前图像特征点的重心
        centroid_curr = compute_centroid(kp_curr)
        if centroid_curr is None:
            centroid_curr = (width // 2, height // 2)

        # 计算旋转角度
        angle = i * theta

        # 绕当前图像的特征点重心旋转
        M = cv2.getRotationMatrix2D(centroid_curr, angle, 1.0)
        rotated_image = cv2.warpAffine(image, M, size)

        # 检测旋转后图像的特征
        kp_rotated, des_rotated = detect_and_compute(rotated_image, orb)

        # 特征匹配
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        if des_rotated is None or des_base is None:
            print(f"图像 {img_path} 特征描述符为空，跳过")
            continue
        matches = bf.match(des_rotated, des_base)
        if len(matches) < 4:
            print(f"图像 {img_path} 匹配点过少，跳过")
            continue

        # 提取匹配的关键点坐标
        src_pts = np.float32([kp_rotated[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_base[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # 计算仿射变换矩阵
        M_affine, mask = cv2.estimateAffine2D(src_pts, dst_pts, method=cv2.RANSAC, ransacReprojThreshold=5.0)
        if M_affine is None:
            print(f"图像 {img_path} 仿射变换估计失败，跳过")
            continue

        # 应用仿射变换对齐图像
        aligned_image = cv2.warpAffine(rotated_image, M_affine, size)
        accumulated_image += aligned_image.astype(np.float32)

    # 计算平均图像
    averaged_image = accumulated_image / num_images
    # 将图像归一化并转换为8位无符号整数
    averaged_image = cv2.normalize(averaged_image, None, 0, 255, cv2.NORM_MINMAX)
    averaged_image = averaged_image.astype(np.uint8)
    # 显示结果
    cv2.namedWindow("叠加结果", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("叠加结果", 1536, 1024)
    cv2.imshow("叠加结果", averaged_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()