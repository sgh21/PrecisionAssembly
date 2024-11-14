import os
import json
import math

def circle_to_polygon(cx, cy, r, num_vertices=36):
    """
    将圆近似为多边形
    参数：
    - cx, cy: 圆心坐标
    - r: 半径
    - num_vertices: 多边形顶点数
    返回：
    - points: 多边形顶点列表，格式为 [(x1, y1), (x2, y2), ...]
    """
    points = []
    for i in range(num_vertices):
        theta = 2 * math.pi * i / num_vertices
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        points.append((x, y))
    return points

def process_json_files(folder_path, output_path, image_width, image_height,seg = True, class_dict=None):
    # 如果没有提供类别字典，初始化一个空字典
    if class_dict is None:
        class_dict = {}
        class_id_counter = 3

    # 获取文件夹中的所有 JSON 文件
    json_files = [f for f in os.listdir(folder_path) if f.endswith('.json')]

    for json_file in json_files:
        json_path = os.path.join(folder_path, json_file)
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        shapes = data.get('shapes', [])
        lines = []
        for shape in shapes:
            label = shape.get('label', '')
            points = shape.get('points', [])
            if len(points) == 2:
                # 确定类别 ID
                if label not in class_dict:
                    class_dict[label] = class_id_counter
                    class_id_counter += 1
                class_id = class_dict[label]
                if seg:
                    # 提取圆心和圆上一点
                    (cx, cy) = points[0]
                    (px, py) = points[1]

                    # 计算半径
                    r = math.hypot(px - cx, py - cy)

                    # 将圆拟合为多边形
                    polygon_points = circle_to_polygon(cx, cy, r)

                    # 归一化多边形顶点坐标
                    normalized_points = [(x / image_width, y / image_height) for x, y in polygon_points]
                    mask_str = ' '.join([f"{x:.6f} {y:.6f}" for x, y in normalized_points])
                    line = f"{class_id} {mask_str}"
                    lines.append(line)
                else :
                    # 计算边界框（左上角和右下角坐标）
                    x_min = cx - r
                    y_min = cy - r
                    x_max = cx + r
                    y_max = cy + r

                    # 确保坐标在图像边界内
                    x_min = max(0, x_min)
                    y_min = max(0, y_min)
                    x_max = min(image_width, x_max)
                    y_max = min(image_height, y_max)

                    # 计算 YOLO 格式的标签
                    x_center = (x_min + x_max) / 2 / image_width
                    y_center = (y_min + y_max) / 2 / image_height
                    width = (x_max - x_min) / image_width
                    height = (y_max - y_min) / image_height

                    # 生成一行文本：<class_id> <x_center> <y_center> <width> <height> <x1> <y1> <x2> <y2> ... <xn> <yn>
                    bbox_str = f"{x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}"
                    line = f"{class_id} {bbox_str}"
                    lines.append(line)
                

        # 将结果保存为同名的 TXT 文件
        txt_file = os.path.splitext(json_file)[0] + '.txt'
        txt_path = os.path.join(output_path, txt_file)
        with open(txt_path, 'w', encoding='utf-8') as f:
            for line in lines:
                f.write(line + '\n')

    # 保存类别映射文件
    classes_path = os.path.join(output_path, 'classes.txt')
    with open(classes_path, 'w', encoding='utf-8') as f:
        # 根据 class_id 排序
        sorted_classes = sorted(class_dict.items(), key=lambda x: x[1])
        for label, class_id in sorted_classes:
            f.write(f"{label}\n")

    print("处理完成。标签文件已生成。")

# 示例用法
if __name__ == "__main__":
    import sys
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace = os.path.dirname(current_dir)

    sys.path.append(workspace)
    # 设置您的 JSON 文件夹路径
    folder_path = f"{workspace}/dataset/circle/0"
    output_path = f"{workspace}/dataset/circle/labels"
    os.makedirs(output_path, exist_ok=True)

    # 设置图像的宽度和高度（根据您的实际图像尺寸）
    image_width = 3072
    image_height = 2048

    process_json_files(folder_path, output_path, image_width, image_height)