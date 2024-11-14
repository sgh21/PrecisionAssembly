import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
def skew(v):
    """
    根据一个 3 元向量创建反对称矩阵（斜对称矩阵）。
    """
    return np.array([
        [0, -v[2], v[1]],
        [v[2],  0, -v[0]],
        [-v[1], v[0],  0]
    ])
def Tsai_HandEye(Hgij_list, Hcij_list):
    """
    使用 Tsai 方法计算手眼标定矩阵 Hcg。

    参数：
    - Hgij_list：包含 (4, 4) 形状的 numpy 数组的列表，每个数组表示变换矩阵 Hgij[i]。
    - Hcij_list：包含 (4, 4) 形状的 numpy 数组的列表，每个数组表示变换矩阵 Hcij[i]。

    返回：
    - Hcg：计算得到的手眼标定矩阵，形状为 (4, 4)。
    """
    assert len(Hgij_list) == len(Hcij_list), "Hgij 和 Hcij 列表的长度必须相同。"
    nStatus = len(Hgij_list)

    A_list = []
    b_list = []

    for i in range(nStatus):
        # 提取旋转矩阵
        Rgij = Hgij_list[i][0:3, 0:3]
        Rcij = Hcij_list[i][0:3, 0:3]

        # 将旋转矩阵转换为旋转向量
        rgij, _ = cv2.Rodrigues(Rgij)
        rcij, _ = cv2.Rodrigues(Rcij)

        # 计算旋转角度
        theta_gij = np.linalg.norm(rgij)
        theta_cij = np.linalg.norm(rcij)

        # 检查旋转角度是否为零
        if theta_gij == 0 or theta_cij == 0:
            print(f"Warning: theta_gij or theta_cij is zero at index {i}. Skipping this iteration.")
            continue

        # 归一化旋转向量
        rngij = rgij / theta_gij if theta_gij != 0 else rgij
        rncij = rcij / theta_cij if theta_cij != 0 else rcij

        # 计算 Pgij 和 Pcij
        Pgij = 2 * np.sin(theta_gij / 2) * rngij
        Pcij = 2 * np.sin(theta_cij / 2) * rncij

        # 计算 tempA 和 tempb
        tempA = skew((Pgij + Pcij).flatten())
        tempb = (Pcij - Pgij).flatten().reshape(3, 1)

        # 添加到列表
        A_list.append(tempA)
        b_list.append(tempb)

    # 堆叠 A 和 b
    A = np.vstack(A_list)
    b = np.vstack(b_list)

    # 计算旋转部分
    pinA = np.linalg.pinv(A)
    Pcg_prime = pinA @ b
    norm_Pcg_prime = np.linalg.norm(Pcg_prime)
    Pcg = 2 * Pcg_prime / np.sqrt(1 + norm_Pcg_prime**2)
    PcgTrs = Pcg.T

    eyeM = np.eye(3)
    norm_Pcg = np.linalg.norm(Pcg)
    Rcg = (1 - norm_Pcg**2 / 2) * eyeM + 0.5 * (Pcg @ PcgTrs + np.sqrt(4 - norm_Pcg**2) * skew(Pcg.flatten()))

    # 计算平移部分
    AA_list = []
    bb_list = []

    for i in range(nStatus):
        Rgij = Hgij_list[i][0:3, 0:3]
        Tgij = Hgij_list[i][0:3, 3].reshape(3, 1)
        Tcij = Hcij_list[i][0:3, 3].reshape(3, 1)

        tempAA = Rgij - eyeM
        tempbb = Rcg @ Tcij - Tgij

        AA_list.append(tempAA)
        bb_list.append(tempbb)

    # 堆叠 AA 和 bb
    AA = np.vstack(AA_list)
    bb = np.vstack(bb_list)

    # 计算 Tcg
    pinAA = np.linalg.pinv(AA)
    Tcg = pinAA @ bb

    # 构建 Hcg
    Hcg = np.eye(4)
    Hcg[0:3, 0:3] = Rcg
    Hcg[0:3, 3] = Tcg.flatten()

    return Hcg


def calculate_homogeneous_transform(v_rot, v_trans):
        """
        计算从相机坐标系到标定板世界坐标系的齐次变换矩阵。

        参数:
        v_rot -- 旋转向量
        v_trans -- 平移向量

        返回:
        T:从标定板世界坐标系到相机坐标系的齐次变换矩阵
        T_inv:从相机坐标系到标定板世界坐标系的齐次变换矩阵
        """
        # 将旋转向量转换为旋转矩阵
        R, _ = cv2.Rodrigues(v_rot)
        
        # 构建齐次变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = v_trans.flatten()
        
        # 计算逆变换
        T_inv = np.linalg.inv(T)
        
        return T,T_inv

def main():
    data_dir = "./calib_data/calib"
    output_dir = f"{data_dir}/output"
    v_trans = np.load(f"{output_dir}/translation.npy")
    v_rot = np.load(f"{output_dir}/rotation.npy")
    ## 构造迭代方程
    T_flange2base = np.load(f"{data_dir}/flange2base.npy")
    if  T_flange2base.shape[0]!= v_trans.shape[0] or \
        T_flange2base.shape[0]!= v_rot.shape[0]:
        raise ValueError("The number of data points does not match!")
    T_calibboard2camera = np.zeros((T_flange2base.shape[0], 4, 4))
    for i in range(T_flange2base.shape[0]):
        T_calibboard2camera[i], _ = calculate_homogeneous_transform(v_rot[i], v_trans[i])
    T_A = np.zeros((T_flange2base.shape[0]-1, 4, 4))
    T_B = np.zeros((T_flange2base.shape[0]-1, 4, 4))
    for i in range(T_flange2base.shape[0]-1):
        T_A[i] = np.linalg.inv(T_flange2base[i+1]) @ T_flange2base[i]  
        T_B[i] = T_calibboard2camera[i+1] @ np.linalg.inv(T_calibboard2camera[i])
    T = Tsai_HandEye(T_A, T_B)
    np.set_printoptions(precision=4, suppress=True)
    rot = T[:3, :3]
    ori = R.from_matrix(rot).as_euler('xyz', degrees=True)
    print(T)
    print(ori)

if __name__ == "__main__":
    main()