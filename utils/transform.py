import numpy as np
from scipy.spatial.transform import Rotation as R


def quaternion_to_standard(quaternion):
    """
    将四元数从 (qw, qx, qy, qz) 转换为  (qx, qy, qz, qw)标准形式。

    参数:
    quaternion -- 四元数 (qw, qx, qy, qz)

    返回:
    标准四元数  (qx, qy, qz, qw)
    """
    if len(quaternion) != 4:
        raise ValueError("quaternion must have 4 elements.")
    return np.array([quaternion[1], quaternion[2],quaternion[3],quaternion[0]])

def standard_to_quaternion(quaternion):
    """
    将四元数从 (qx, qy, qz, qw) 转换为  (qw, qx, qy, qz)标准形式。

    参数:
    quaternion -- 四元数 (qx, qy, qz, qw)

    返回:
    标准四元数  (qw, qx, qy, qz)
    """
    if len(quaternion) != 4:
        raise ValueError("quaternion must have 4 elements.")
    return np.array([quaternion[3], quaternion[0], quaternion[1], quaternion[2]])

def rpy_to_quaternion(rpy):
    """
    将 Roll-Pitch-Yaw (RPY) 角转换为四元数。

    参数:
    rpy -- 包含 roll, pitch, yaw 的数组（弧度）

    返回:
    四元数 (qx, qy, qz, qw)
    """
    if len(rpy) != 3:
        raise ValueError("RPY angles must have 3 elements.")
    # zyx的欧拉角对应于roll pitch yaw
    # 这个欧拉角默认是外旋的
    r = R.from_euler('xyz', rpy)
    quaternion = r.as_quat()
    
    return quaternion

def quaternion_to_rpy(quaternion):
    """
    将四元数转换为 Roll-Pitch-Yaw (RPY) 角。

    参数:
    quaternion -- 四元数 (qx, qy, qz, qw)

    返回:
    RPY 角 (roll, pitch, yaw)
    """
    if len(quaternion) != 4:
        raise ValueError("quaternion must have 4 elements.")
    
    r = R.from_quat(quaternion)
    rpy = r.as_euler('xyz', degrees=False)
    
    return rpy

if __name__ == "__main__":
    # 示例调用
    roll = np.radians(30)  # 30 度转换为弧度
    pitch = np.radians(45)  # 45 度转换为弧度
    yaw = np.radians(75)  # 60 度转换为弧度
    #### 还需要对比函数输出的四元数和机器人四元数
    qx, qy, qz, qw = rpy_to_quaternion(np.array([ roll, pitch, yaw ]))
    quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    print(f"Quaternion: ({qx}, {qy}, {qz}, {qw})")
    r,p,y = quaternion_to_rpy(np.array([qx, qy, qz, qw]))
    print(f"RPY: ({r}, {p}, {y})")
    print(f"RPY: ({np.degrees(r)}, {np.degrees(p)}, {np.degrees(y)})")

    