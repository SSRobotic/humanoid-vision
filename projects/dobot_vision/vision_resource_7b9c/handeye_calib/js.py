import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse
import matplotlib.pyplot as plt


def load_data(file_path):
    """加载YAML格式的标定数据"""
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    return data[next(iter(data))]  # 获取第一个键的值


def preprocess_aruco(data):
    """预处理Aruco数据：提取位置和旋转矩阵"""
    positions = []
    rotations = []
    for point in data:
        # 提取位置 (x, y, z)
        pos = np.array(point[:3])
        positions.append(pos)

        # 提取四元数并转换为旋转矩阵
        quat = np.array(point[3:])  # [qx, qy, qz, qw]
        rotations.append(R.from_quat(quat).as_matrix())
    return positions, rotations


def preprocess_dobot(data, euler_order='zyx'):
    """预处理Dobot数据：提取位置和旋转矩阵"""
    positions = []
    rotations = []
    for point in data:
        # 提取位置 (x, y, z)
        pos = np.array(point[:3])
        positions.append(pos)

        # 提取欧拉角并转换为旋转矩阵
        euler_deg = np.array(point[3:])
        euler_rad = np.radians(euler_deg)  # 度转弧度

        # 尝试不同的欧拉角顺序
        try:
            rotations.append(R.from_euler(euler_order, euler_rad).as_matrix())
        except:
            # 如果顺序错误，尝试反转
            rotations.append(R.from_euler(euler_order[::-1], euler_rad).as_matrix())
    return positions, rotations


def filter_outliers(positions, rotations, threshold=50):
    """基于移动距离过滤异常点"""
    if len(positions) < 2:
        return positions, rotations

    # 计算相邻点之间的距离
    distances = [np.linalg.norm(np.array(positions[i]) - np.array(positions[i - 1]))
                 for i in range(1, len(positions))]

    # 找出正常移动范围
    median_dist = np.median(distances)
    valid_idx = [0]  # 保留第一个点

    # 筛选移动距离在合理范围内的点
    for i in range(1, len(positions)):
        dist = np.linalg.norm(positions[i] - positions[i - 1])
        if median_dist * 0.1 < dist < median_dist * 10:
            valid_idx.append(i)

    # 返回筛选后的数据
    return [positions[i] for i in valid_idx], [rotations[i] for i in valid_idx]


def calibrate_hand_eye(A_pos, A_rot, B_pos, B_rot):
    """手眼标定算法实现 (Tsai-Len方法)"""
    n = len(A_rot) - 1
    if n < 3:
        raise ValueError("至少需要4组数据点进行标定")

    # 构建旋转和平移方程组
    C = np.zeros((3 * n, 3))
    d = np.zeros((3 * n, 1))  # 保持为列向量
    H = np.zeros((3 * n, 3))
    b = np.zeros((3 * n, 1))  # 保持为列向量

    for i in range(n):
        # 计算相对旋转
        RA = A_rot[i].T @ A_rot[i + 1]
        RB = B_rot[i].T @ B_rot[i + 1]

        # 计算相对平移
        PA = A_rot[i].T @ (A_pos[i + 1] - A_pos[i])
        PB = B_rot[i].T @ (B_pos[i + 1] - B_pos[i])

        # 构建旋转方程 - 修复形状问题
        C[3 * i:3 * i + 3] = np.eye(3) - RA
        d[3 * i:3 * i + 3, 0] = (RB - np.eye(3)) @ PB - PA  # 确保正确赋值

        # 构建平移方程 - 修复形状问题
        H[3 * i:3 * i + 3] = RA - np.eye(3)
        b[3 * i:3 * i + 3, 0] = PA  # 确保正确赋值

    # 求解旋转矩阵
    RX = np.linalg.lstsq(C, d, rcond=None)[0]

    # 使用SVD分解确保旋转矩阵正交性
    U, S, Vt = np.linalg.svd(RX)
    RX_ortho = U @ Vt

    # 求解平移向量
    tX = np.linalg.lstsq(H, b - (H @ RX_ortho), rcond=None)[0]

    return RX_ortho, tX.flatten()


def evaluate_calibration(R_x, t_x, aruco_pos, aruco_rot, dobot_pos, dobot_rot):
    """评估标定结果"""
    pos_errors = []
    rot_errors = []

    for i in range(len(aruco_pos)):
        # 将Aruco位姿转换到Dobot坐标系
        transformed_pos = R_x @ aruco_pos[i] + t_x
        transformed_rot = R_x @ aruco_rot[i]

        # 计算位置误差 (mm)
        pos_error = np.linalg.norm(transformed_pos - dobot_pos[i])
        pos_errors.append(pos_error)

        # 计算旋转误差 (度)
        rel_rot = transformed_rot.T @ dobot_rot[i]
        rot_error = R.from_matrix(rel_rot).magnitude() * 180 / np.pi
        rot_errors.append(rot_error)

    return pos_errors, rot_errors


def plot_errors(pos_errors, rot_errors):
    """可视化误差分布"""
    plt.figure(figsize=(12, 6))

    # 位置误差分布
    plt.subplot(1, 2, 1)
    plt.hist(pos_errors, bins=20, color='skyblue', edgecolor='black')
    plt.axvline(np.mean(pos_errors), color='r', linestyle='dashed', linewidth=2)
    plt.title(f'Position Errors\nMean: {np.mean(pos_errors):.2f} mm')
    plt.xlabel('Error (mm)')
    plt.ylabel('Count')

    # 旋转误差分布
    plt.subplot(1, 2, 2)
    plt.hist(rot_errors, bins=20, color='lightgreen', edgecolor='black')
    plt.axvline(np.mean(rot_errors), color='r', linestyle='dashed', linewidth=2)
    plt.title(f'Rotation Errors\nMean: {np.mean(rot_errors):.2f} deg')
    plt.xlabel('Error (degrees)')

    plt.tight_layout()
    plt.savefig('calibration_errors.png')
    plt.show()


def main(aruco_file, dobot_file, euler_order='zyx', filter_data=True):
    """主函数：执行完整标定流程"""
    # 加载数据
    aruco_data = load_data(aruco_file)
    dobot_data = load_data(dobot_file)

    # 确保数据点数量一致
    min_len = min(len(aruco_data), len(dobot_data))
    aruco_data = aruco_data[:min_len]
    dobot_data = dobot_data[:min_len]

    print(f"使用 {min_len} 组数据点进行标定...")

    # 预处理数据
    aruco_pos, aruco_rot = preprocess_aruco(aruco_data)
    dobot_pos, dobot_rot = preprocess_dobot(dobot_data, euler_order)

    # 过滤异常点
    if filter_data:
        aruco_pos, aruco_rot = filter_outliers(aruco_pos, aruco_rot)
        dobot_pos, dobot_rot = filter_outliers(dobot_pos, dobot_rot)
        print(f"过滤后保留 {len(aruco_pos)} 组有效数据点")

    # 执行手眼标定
    try:
        R_x, t_x = calibrate_hand_eye(
            A_pos=aruco_pos,
            A_rot=aruco_rot,
            B_pos=dobot_pos,
            B_rot=dobot_rot
        )

        print("\n标定结果:")
        print(f"旋转矩阵:\n{R_x}")
        print(f"平移向量: {t_x}")

        # 评估标定结果
        pos_errors, rot_errors = evaluate_calibration(
            R_x, t_x, aruco_pos, aruco_rot, dobot_pos, dobot_rot
        )

        print("\n验证结果:")
        print(f"平均位置误差: {np.mean(pos_errors):.4f} mm")
        print(f"最大位置误差: {np.max(pos_errors):.4f} mm")
        print(f"平均角度误差: {np.mean(rot_errors):.4f} 度")
        print(f"最大角度误差: {np.max(rot_errors):.4f} 度")

        # 可视化误差分布
        plot_errors(pos_errors, rot_errors)

    except Exception as e:
        print(f"标定失败: {str(e)}")
        print("尝试以下解决方案:")
        print("1. 检查欧拉角顺序参数 (--euler_order)")
        print("2. 增加数据点数量")
        print("3. 确保机械臂有足够的旋转和平移运动")
        print("4. 禁用异常点过滤 (--no_filter)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='手眼标定系统')
    parser.add_argument('--aruco', type=str, default='aruco_p.yaml',
                        help='Aruco数据文件路径')
    parser.add_argument('--dobot', type=str, default='dobot_p.yaml',
                        help='Dobot数据文件路径')
    parser.add_argument('--euler_order', type=str, default='zyx',
                        help='Dobot欧拉角顺序 (默认: zyx)')
    parser.add_argument('--no_filter', action='store_true',
                        help='禁用异常点过滤')

    args = parser.parse_args()

    main(
        aruco_file=args.aruco,
        dobot_file=args.dobot,
        euler_order=args.euler_order,
        filter_data=not args.no_filter
    )