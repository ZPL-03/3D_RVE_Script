# -*- coding: utf-8 -*-
# ##################################################################
#
#           可控制纤维体积分数的Abaqus三维RVE建模脚本
#
# 功能说明:
# 1. 根据用户设定的纤维体积分数(Vf)自动生成三维RVE几何模型
# 2. 采用混合算法（RSA播种 + 锚定松弛 + 强制校正）排布纤维，兼顾速度与分布质量
# 3. 纤维沿Z轴方向为连续圆柱体，XY截面满足周期性分布
# 4. 保证纤维间的最小间距（仅XY平面），无法满足时报错终止
# 5. 自动创建材料（基体+纤维[正交各向异性]+界面）、赋予截面、划分网格
# 6. 施加三维周期性边界条件(PBCs)实现XYZ三方向周期性
# 7. 在纤维-基体界面插入三维cohesive单元(COH3D8)
# 8. 利用几何位置算法准确识别纤维体和基体体（包括边角碎片）
# 9. 纤维材料方向自动对齐至Z轴（Direction-1沿+Z）
# 10. 导出纤维中心坐标为CSV文件，便于验证和后续分析
# 11. 验证纤维间距并输出详细统计信息
#
# 与2D版本的主要区别:
# - 纤维形状：2D为圆形截面，3D为沿Z轴的连续圆柱体
# - 材料模型：3D纤维采用正交各向异性，2D为各向同性
# - PBCs：3D需要XYZ三个方向的周期性约束，2D仅需XY
# - Cohesive单元：3D使用COH3D8，2D使用COH2D4
# - 距离验证：3D仅验证XY平面距离（因纤维沿Z连续）
# - 网格复杂度：3D网格量约为2D的10-50倍，计算成本显著增加
#
# 作者: 刘正鹏 (Liu Zhengpeng)
# 版本: v1.0
# 创建日期: 2025-09-30
# 最后更新: 2025-XX-XX
# 适用软件: ABAQUS 2023
# Python版本: 2.7 (ABAQUS内置)
# 技术交流: GitHub/CSDN/知乎 @小盆i
# 联系方式: 1370872708@qq.com / Zhengpeng0105@gmail.com
#
# ##################################################################

from abaqus import *
from abaqusConstants import *
from caeModules import *
from driverUtils import executeOnCaeStartup
import math
import random as rd
import time
import os

from part import *
from material import *
from section import *
from sketch import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from visualization import *
from connectorBehavior import *

executeOnCaeStartup()


# =================================================================
#                 CSV坐标输出模块
# =================================================================
def exportFiberCentersToCSV(fiber_centers, filename, rveSize, fiberRadius, depth, target_Vf):
    """将纤维中心坐标导出为CSV文件

    参数:
        fiber_centers: 纤维中心坐标列表 [(x1,y1), (x2,y2), ...]
        filename: 输出的CSV文件名
        rveSize: RVE尺寸 [宽度, 高度, 深度]
        fiberRadius: 纤维半径
        depth: 纤维长度(沿z方向)
        target_Vf: 目标体积分数

    返回:
        filepath: 成功则返回文件完整路径,失败则返回None

    注意:
        - 导出的坐标为RVE内的有效纤维中心,不包括周期性重复的纤维
        - 三维纤维为沿z方向的连续圆柱体
    """
    try:
        work_dir = os.getcwd()
        filepath = os.path.join(work_dir, filename)

        with open(filepath, 'w') as f:
            f.write("# 3D RVE Fiber Center Coordinates\n")
            f.write("# Generated: %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
            f.write("# RVE Size (Width x Height x Depth): %.6f x %.6f x %.6f\n" %
                    (rveSize[0], rveSize[1], depth))
            f.write("# Fiber Radius: %.6f\n" % fiberRadius)
            f.write("# Fiber Length: %.6f\n" % depth)
            f.write("# Target Volume Fraction: %.4f\n" % target_Vf)
            f.write("# Total Fiber Count: %d\n" % len(fiber_centers))
            f.write("#\n")
            f.write("Fiber_ID,X_Coordinate,Y_Coordinate,Z_Start,Z_End\n")

            for i, (x, y) in enumerate(fiber_centers, start=1):
                f.write("%d,%.8f,%.8f,%.8f,%.8f\n" % (i, x, y, 0.0, depth))

        print("\n" + "=" * 60)
        print("SUCCESS: Fiber coordinates exported to CSV")
        print("File location: %s" % filepath)
        print("Total fibers: %d" % len(fiber_centers))
        print("=" * 60 + "\n")
        return filepath

    except Exception as e:
        print("\nWARNING: Failed to export coordinates: %s\n" % str(e))
        return None


# =================================================================
#                 周期性边界条件 (PBCs) 辅助函数
# =================================================================

def createReferencePoints3D(model):
    """创建三个参考点用于三维周期性边界条件

    参数:
        model: Abaqus模型对象

    功能:
        创建Ref-X、Ref-Y和Ref-Z三个参考点,分别用于:
        - Ref-X: 左右边界(X方向)的周期性约束
        - Ref-Y: 前后边界(Y方向)的周期性约束
        - Ref-Z: 上下边界(Z方向)的周期性约束
    """
    rootAssembly = model.rootAssembly

    p_X = model.Part(name='Ref-X-Part', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p_X.ReferencePoint(point=(0.0, 0.0, 0.0))

    p_Y = model.Part(name='Ref-Y-Part', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p_Y.ReferencePoint(point=(0.0, 0.0, 0.0))

    p_Z = model.Part(name='Ref-Z-Part', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p_Z.ReferencePoint(point=(0.0, 0.0, 0.0))

    rootAssembly.Instance(name='Ref-X-Instance', part=p_X, dependent=ON)
    rootAssembly.Instance(name='Ref-Y-Instance', part=p_Y, dependent=ON)
    rootAssembly.Instance(name='Ref-Z-Instance', part=p_Z, dependent=ON)

    rootAssembly.Set(name='set_RefPoint_X',
                     referencePoints=(rootAssembly.instances['Ref-X-Instance'].referencePoints[1],))
    rootAssembly.Set(name='set_RefPoint_Y',
                     referencePoints=(rootAssembly.instances['Ref-Y-Instance'].referencePoints[1],))
    rootAssembly.Set(name='set_RefPoint_Z',
                     referencePoints=(rootAssembly.instances['Ref-Z-Instance'].referencePoints[1],))


def getRVEDimensions3D(model, instanceName):
    """获取三维RVE的边界坐标

    参数:
        model: Abaqus模型对象
        instanceName: 实例名称

    返回:
        (xMin, xMax, yMin, yMax, zMin, zMax): RVE的边界坐标
    """
    nodes = model.rootAssembly.instances[instanceName].nodes

    xMin = min(n.coordinates[0] for n in nodes)
    xMax = max(n.coordinates[0] for n in nodes)
    yMin = min(n.coordinates[1] for n in nodes)
    yMax = max(n.coordinates[1] for n in nodes)
    zMin = min(n.coordinates[2] for n in nodes)
    zMax = max(n.coordinates[2] for n in nodes)

    return xMin, xMax, yMin, yMax, zMin, zMax


def getBoundaryNodes3D(model, instanceName, dimensions):
    """获取六个面上的所有节点

    参数:
        model: Abaqus模型对象
        instanceName: 实例名称
        dimensions: 边界尺寸 (xMin, xMax, yMin, yMax, zMin, zMax)

    返回:
        六元组: (nodes_left, nodes_right, nodes_front, nodes_back, nodes_bottom, nodes_top)
        - nodes_left: 左侧面(x=xMin)的节点列表
        - nodes_right: 右侧面(x=xMax)的节点列表
        - nodes_front: 前侧面(y=yMin)的节点列表
        - nodes_back: 后侧面(y=yMax)的节点列表
        - nodes_bottom: 底面(z=zMin)的节点列表
        - nodes_top: 顶面(z=zMax)的节点列表
    """
    nodes = model.rootAssembly.instances[instanceName].nodes
    xMin, xMax, yMin, yMax, zMin, zMax = dimensions
    tol = 1e-6

    nodes_left, nodes_right = [], []
    nodes_front, nodes_back = [], []
    nodes_bottom, nodes_top = [], []

    for n in nodes:
        x, y, z = n.coordinates[0], n.coordinates[1], n.coordinates[2]

        if abs(x - xMin) < tol: nodes_left.append(n)
        if abs(x - xMax) < tol: nodes_right.append(n)
        if abs(y - yMin) < tol: nodes_front.append(n)
        if abs(y - yMax) < tol: nodes_back.append(n)
        if abs(z - zMin) < tol: nodes_bottom.append(n)
        if abs(z - zMax) < tol: nodes_top.append(n)

    # 对节点进行排序,便于配对
    nodes_left.sort(key=lambda n: (n.coordinates[1], n.coordinates[2]))
    nodes_right.sort(key=lambda n: (n.coordinates[1], n.coordinates[2]))
    nodes_front.sort(key=lambda n: (n.coordinates[0], n.coordinates[2]))
    nodes_back.sort(key=lambda n: (n.coordinates[0], n.coordinates[2]))
    nodes_bottom.sort(key=lambda n: (n.coordinates[0], n.coordinates[1]))
    nodes_top.sort(key=lambda n: (n.coordinates[0], n.coordinates[1]))

    return nodes_left, nodes_right, nodes_front, nodes_back, nodes_bottom, nodes_top


def pairBoundaryNodes3D(slave_nodes, master_nodes, tolerance, coord_indices):
    """三维节点配对算法

    参数:
        slave_nodes: 从属边界的节点列表
        master_nodes: 主边界的节点列表
        tolerance: 配对容差
        coord_indices: 配对时使用的坐标索引元组 (idx1, idx2)
                       例如: (1,2)表示使用y和z坐标进行配对

    返回:
        paired_nodes: 配对结果列表 [(slave_node1, master_node1), ...]

    功能:
        优先匹配平面内距离最近的节点,用于三维周期性边界条件
    """
    paired_nodes = []
    master_pool = list(master_nodes)
    slave_pool = list(slave_nodes)

    idx1, idx2 = coord_indices

    for s_node in slave_pool:
        s_coord = s_node.coordinates
        best_match_node = None
        min_dist = float('inf')

        for m_node in master_pool:
            m_coord = m_node.coordinates

            delta1 = abs(s_coord[idx1] - m_coord[idx1])
            delta2 = abs(s_coord[idx2] - m_coord[idx2])

            planar_dist = math.sqrt(delta1 ** 2 + delta2 ** 2)

            if planar_dist < tolerance and planar_dist < min_dist:
                min_dist = planar_dist
                best_match_node = m_node

        if best_match_node:
            paired_nodes.append((s_node, best_match_node))
            master_pool.remove(best_match_node)

    return paired_nodes


def applyPeriodicConstraints3D(model, instanceName, node_pairs, pair_type):
    """施加三维周期性约束

    参数:
        model: Abaqus模型对象
        instanceName: 实例名称
        node_pairs: 节点配对列表 [(node1, node2), ...]
        pair_type: 配对类型 ('Left-Right', 'Front-Back', 或 'Bottom-Top')

    功能:
        为每对节点创建约束方程: u_slave - u_master + u_ref = 0
        对X、Y、Z三个方向分别创建约束方程
    """
    r_assy = model.rootAssembly
    inst = r_assy.instances[instanceName]

    if pair_type == 'Left-Right':
        ref_point_name = 'set_RefPoint_X'
        tag1, tag2 = 'L', 'R'
    elif pair_type == 'Front-Back':
        ref_point_name = 'set_RefPoint_Y'
        tag1, tag2 = 'F', 'B'
    elif pair_type == 'Bottom-Top':
        ref_point_name = 'set_RefPoint_Z'
        tag1, tag2 = 'Bo', 'T'
    else:
        return

    coeffs = (1.0, -1.0, 1.0)

    for i, (node1, node2) in enumerate(node_pairs):
        set1_name = 'set_Node-%s-%d' % (tag1, i + 1)
        set2_name = 'set_Node-%s-%d' % (tag2, i + 1)

        r_assy.Set(nodes=inst.nodes.sequenceFromLabels(labels=(node1.label,)), name=set1_name)
        r_assy.Set(nodes=inst.nodes.sequenceFromLabels(labels=(node2.label,)), name=set2_name)

        # X方向约束
        model.Equation(name='Eq-%s%s-X-%d' % (tag1, tag2, i + 1),
                       terms=((coeffs[0], set1_name, 1),
                              (coeffs[1], set2_name, 1),
                              (coeffs[2], ref_point_name, 1)))

        # Y方向约束
        model.Equation(name='Eq-%s%s-Y-%d' % (tag1, tag2, i + 1),
                       terms=((coeffs[0], set1_name, 2),
                              (coeffs[1], set2_name, 2),
                              (coeffs[2], ref_point_name, 2)))

        # Z方向约束
        model.Equation(name='Eq-%s%s-Z-%d' % (tag1, tag2, i + 1),
                       terms=((coeffs[0], set1_name, 3),
                              (coeffs[1], set2_name, 3),
                              (coeffs[2], ref_point_name, 3)))


# =================================================================
#                 纤维排布算法模块
# =================================================================
def _relax_coords_anchored(initial_coords, seeding_count, fiberCount,
                           rveSize, fiberRadius, minDistance):
    """锚定松弛算法

    参数:
        initial_coords: 初始纤维坐标列表
        seeding_count: RSA播种的纤维数量(这些纤维作为锚点,移动受限)
        fiberCount: 总纤维数量
        rveSize: RVE尺寸 [宽度, 高度, 深度]
        fiberRadius: 纤维半径
        minDistance: 最小间距

    返回:
        coords: 松弛后的纤维坐标列表

    功能:
        通过模拟斥力来调整纤维位置,锚点纤维的移动受到阻尼限制
    """
    print("--- Initializing Anchored Relaxation Process ---")

    coords = [list(c) for c in initial_coords]
    max_iterations = 2000
    movement_factor = 0.5
    anchor_damping_factor = 0.05
    min_dist_sq = minDistance ** 2

    for iter_num in range(max_iterations):
        max_movement_sq = 0.0
        net_forces = [[0.0, 0.0] for _ in range(fiberCount)]

        for i in range(fiberCount):
            for j in range(i + 1, fiberCount):
                dx = coords[j][0] - coords[i][0]
                dy = coords[j][1] - coords[i][1]

                # 周期性边界条件修正
                if dx > rveSize[0] / 2: dx -= rveSize[0]
                if dx < -rveSize[0] / 2: dx += rveSize[0]
                if dy > rveSize[1] / 2: dy -= rveSize[1]
                if dy < -rveSize[1] / 2: dy += rveSize[1]

                dist_sq = dx * dx + dy * dy

                if dist_sq < min_dist_sq:
                    dist = math.sqrt(dist_sq) if dist_sq > 0 else 1e-9
                    overlap = minDistance - dist
                    force_magnitude = overlap

                    force_x = force_magnitude * (dx / dist)
                    force_y = force_magnitude * (dy / dist)

                    net_forces[i][0] -= force_x
                    net_forces[i][1] -= force_y
                    net_forces[j][0] += force_x
                    net_forces[j][1] += force_y

        if not any(any(f) for f in net_forces):
            print("--- System stable. No overlaps detected. ---")
            return coords

        for i in range(fiberCount):
            move_x = net_forces[i][0] * movement_factor
            move_y = net_forces[i][1] * movement_factor

            # 锚点纤维的移动受阻尼限制
            if i < seeding_count:
                move_x *= anchor_damping_factor
                move_y *= anchor_damping_factor

            coords[i][0] += move_x
            coords[i][1] += move_y

            # 周期性边界条件
            coords[i][0] %= rveSize[0]
            coords[i][1] %= rveSize[1]

            current_movement_sq = move_x ** 2 + move_y ** 2
            if current_movement_sq > max_movement_sq:
                max_movement_sq = current_movement_sq

        if (iter_num + 1) % 50 == 0:
            print("... Relaxation Iteration %d, Max movement: %.2e" %
                  (iter_num + 1, math.sqrt(max_movement_sq)))

        if max_movement_sq < (1e-6 * fiberRadius) ** 2:
            print("--- Converged after %d iterations. ---" % (iter_num + 1))
            return coords

    print("--- Max relaxation iterations reached. ---")
    return coords


def _final_check_and_enforce(coords_in, fiberCount, rveSize, minDistance):
    """最终检查和强制校正

    参数:
        coords_in: 输入的纤维坐标列表
        fiberCount: 纤维数量
        rveSize: RVE尺寸 [宽度, 高度, 深度]
        minDistance: 最小间距

    返回:
        coords: 校正后的坐标列表

    功能:
        检查所有纤维对的间距,对违反最小间距要求的纤维进行强制移动
        如果无法满足最小间距要求,抛出异常
    """
    coords = [list(c) for c in coords_in]
    min_dist_sq = minDistance ** 2
    max_correction_iter = 50000

    for iter_num in range(max_correction_iter):
        min_dist_sq_found = float('inf')
        worst_offenders = None

        for i in range(fiberCount):
            for j in range(i + 1, fiberCount):
                dx = coords[j][0] - coords[i][0]
                dy = coords[j][1] - coords[i][1]

                if dx > rveSize[0] / 2: dx -= rveSize[0]
                if dx < -rveSize[0] / 2: dx += rveSize[0]
                if dy > rveSize[1] / 2: dy -= rveSize[1]
                if dy < -rveSize[1] / 2: dy += rveSize[1]

                dist_sq = dx * dx + dy * dy

                if dist_sq < min_dist_sq_found:
                    min_dist_sq_found = dist_sq
                    worst_offenders = (i, j, dx, dy)

        if min_dist_sq_found >= min_dist_sq:
            print("--- Final Check PASSED after %d iterations. ---" % (iter_num + 1))
            return coords

        if worst_offenders:
            i, j, dx, dy = worst_offenders
            dist = math.sqrt(min_dist_sq_found) if min_dist_sq_found > 0 else 1e-9
            overlap = minDistance - dist
            move_dist = overlap / 2.0 + 1e-8

            move_x = move_dist * (dx / dist)
            move_y = move_dist * (dy / dist)

            coords[i][0] -= move_x
            coords[i][1] -= move_y
            coords[j][0] += move_x
            coords[j][1] += move_y

            coords[i][0] %= rveSize[0]
            coords[i][1] %= rveSize[1]
            coords[j][0] %= rveSize[0]
            coords[j][1] %= rveSize[1]

        if (iter_num + 1) % 100 == 0:
            print("... Correction Iteration %d, Min distance: %.6f (Target: %.6f)" %
                  (iter_num + 1, math.sqrt(min_dist_sq_found), minDistance))

    # 最终验证
    final_min_dist_sq = float('inf')
    violating_pairs = 0
    violating_fibers = set()

    for i in range(fiberCount):
        for j in range(i + 1, fiberCount):
            dx = coords[j][0] - coords[i][0]
            dy = coords[j][1] - coords[i][1]
            if dx > rveSize[0] / 2: dx -= rveSize[0]
            if dx < -rveSize[0] / 2: dx += rveSize[0]
            if dy > rveSize[1] / 2: dy -= rveSize[1]
            if dy < -rveSize[1] / 2: dy += rveSize[1]
            dist_sq = dx * dx + dy * dy

            if dist_sq < final_min_dist_sq:
                final_min_dist_sq = dist_sq

            if dist_sq < min_dist_sq:
                violating_pairs += 1
                violating_fibers.add(i)
                violating_fibers.add(j)

    if final_min_dist_sq < min_dist_sq:
        error_msg = (
                "FATAL ERROR: Cannot satisfy minimum distance after %d iterations.\n"
                " -> Final min distance: %.6f (Target: >= %.6f)\n"
                " -> Violating pairs: %d, Involved fibers: %d\n"
                " -> Please reduce Vf or increase RVE size."
                % (max_correction_iter, math.sqrt(final_min_dist_sq), minDistance,
                   violating_pairs, len(violating_fibers))
        )
        raise Exception(error_msg)
    else:
        print("--- Final Check PASSED. ---")
        return coords


# =================================================================
#         三维纤维间距验证模块
# =================================================================
def verifyMinimumFiberDistance3D(fiber_centers, rveSize, fiberRadius, minDistanceFactor):
    """验证所有纤维对之间的距离是否满足最小间距要求

    参数:
        fiber_centers: 纤维中心坐标列表 [(x1,y1), (x2,y2), ...]
        rveSize: RVE尺寸 [宽度, 高度, 深度]
        fiberRadius: 纤维半径
        minDistanceFactor: 最小间距因子(例如2.05)

    返回:
        verification_passed: 布尔值,是否通过验证
        stats: 统计信息字典

    注意:
        三维RVE中纤维沿Z方向连续,因此只需验证XY平面内的距离
    """
    print("\n" + "=" * 70)
    print("FIBER DISTANCE VERIFICATION (3D)")
    print("=" * 70)

    if len(fiber_centers) == 0:
        print("No fibers to verify.")
        return True, {}

    if len(fiber_centers) == 1:
        print("Only 1 fiber - no distance check needed.")
        return True, {'fiber_count': 1}

    fiberCount = len(fiber_centers)
    minDistance = minDistanceFactor * fiberRadius

    print("\nConfiguration:")
    print("  Total Fibers: %d" % fiberCount)
    print("  Fiber Radius: %.6f mm" % fiberRadius)
    print("  Min Distance Factor: %.2f" % minDistanceFactor)
    print("  Required Min Distance: %.6f mm (%.2f * %.6f)" %
          (minDistance, minDistanceFactor, fiberRadius))
    print("  RVE Size: %.6f * %.6f * %.6f mm" % (rveSize[0], rveSize[1], rveSize[2]))

    print("\nCalculating inter-fiber distances (in XY plane)...")

    min_distance_found = float('inf')
    max_distance_found = 0.0
    total_pairs = 0
    violating_pairs = []
    all_distances = []

    for i in range(fiberCount):
        for j in range(i + 1, fiberCount):
            # 考虑XY平面的周期性边界条件
            dx = fiber_centers[j][0] - fiber_centers[i][0]
            dy = fiber_centers[j][1] - fiber_centers[i][1]

            if dx > rveSize[0] / 2.0:
                dx -= rveSize[0]
            if dx < -rveSize[0] / 2.0:
                dx += rveSize[0]
            if dy > rveSize[1] / 2.0:
                dy -= rveSize[1]
            if dy < -rveSize[1] / 2.0:
                dy += rveSize[1]

            distance = math.sqrt(dx * dx + dy * dy)
            all_distances.append(distance)
            total_pairs += 1

            if distance < min_distance_found:
                min_distance_found = distance
            if distance > max_distance_found:
                max_distance_found = distance

            if distance < minDistance:
                violating_pairs.append({
                    'fiber_i': i + 1,
                    'fiber_j': j + 1,
                    'distance': distance,
                    'violation': minDistance - distance,
                    'center_i': fiber_centers[i],
                    'center_j': fiber_centers[j]
                })

    avg_distance = sum(all_distances) / len(all_distances)
    all_distances.sort()
    median_distance = all_distances[len(all_distances) // 2]

    print("\n" + "-" * 70)
    print("DISTANCE STATISTICS")
    print("-" * 70)
    print("  Total Fiber Pairs Checked: %d" % total_pairs)
    print("  Minimum Distance Found: %.6f mm" % min_distance_found)
    print("  Maximum Distance Found: %.6f mm" % max_distance_found)
    print("  Average Distance: %.6f mm" % avg_distance)
    print("  Median Distance: %.6f mm" % median_distance)

    distance_ratio = min_distance_found / minDistance
    print("\n  Distance Ratio (min_found / required): %.4f" % distance_ratio)

    if distance_ratio >= 1.0:
        print("  Status: PASSED (%.2f%% above requirement)" %
              ((distance_ratio - 1.0) * 100))
    else:
        print("  Status: FAILED (%.2f%% below requirement)" %
              ((1.0 - distance_ratio) * 100))

    print("\n" + "-" * 70)
    if len(violating_pairs) == 0:
        print("VERIFICATION RESULT: ALL DISTANCES SATISFY MINIMUM REQUIREMENT")
        print("-" * 70)
        verification_passed = True
    else:
        print("VERIFICATION RESULT: MINIMUM DISTANCE VIOLATIONS DETECTED")
        print("-" * 70)
        print("  Number of Violating Pairs: %d" % len(violating_pairs))
        print("\nTop 10 Violations (sorted by severity):")
        print("-" * 70)

        violating_pairs.sort(key=lambda x: x['violation'], reverse=True)

        for idx, violation in enumerate(violating_pairs[:10], 1):
            print("\n  Violation #%d:" % idx)
            print("    Fiber Pair: #%d <-> #%d" %
                  (violation['fiber_i'], violation['fiber_j']))
            print("    Actual Distance: %.6f mm" % violation['distance'])
            print("    Required Distance: %.6f mm" % minDistance)
            print("    Shortfall: %.6f mm (%.2f%% below requirement)" %
                  (violation['violation'],
                   violation['violation'] / minDistance * 100))
            print("    Center #%d: (%.6f, %.6f)" %
                  (violation['fiber_i'],
                   violation['center_i'][0],
                   violation['center_i'][1]))
            print("    Center #%d: (%.6f, %.6f)" %
                  (violation['fiber_j'],
                   violation['center_j'][0],
                   violation['center_j'][1]))

        if len(violating_pairs) > 10:
            print("\n  ... and %d more violations" %
                  (len(violating_pairs) - 10))

        verification_passed = False

    print("=" * 70 + "\n")

    stats = {
        'fiber_count': fiberCount,
        'total_pairs': total_pairs,
        'min_distance': min_distance_found,
        'max_distance': max_distance_found,
        'avg_distance': avg_distance,
        'median_distance': median_distance,
        'required_distance': minDistance,
        'distance_ratio': distance_ratio,
        'violations_count': len(violating_pairs),
        'violations': violating_pairs
    }

    return verification_passed, stats


# =================================================================
#         改进的三维纤维-基体分类算法
# =================================================================
def buildAllFiberCenters3D(fiber_centers, rveSize, fiberRadius):
    """构建包含周期性镜像的完整纤维中心列表

    参数:
        fiber_centers: 原始RVE内的纤维中心列表 [(x1,y1), (x2,y2), ...]
        rveSize: RVE尺寸 [width, height, depth]
        fiberRadius: 纤维半径

    返回:
        unique_centers: 包含所有周期性镜像的纤维中心列表

    功能:
        为靠近XY平面边界的纤维创建周期性镜像,用于准确判断体(cell)的归属

    注意:
        三维纤维沿Z方向是连续的圆柱体,只需考虑XY平面的周期性
        对于边角处的纤维,需要创建对角镜像以确保分类准确性
    """
    all_fiber_centers = []

    for xt, yt in fiber_centers:
        # 原始位置
        all_fiber_centers.append((xt, yt))

        # 左右周期性(靠近X边界的纤维)
        if xt < fiberRadius:
            all_fiber_centers.append((xt + rveSize[0], yt))
        if xt > rveSize[0] - fiberRadius:
            all_fiber_centers.append((xt - rveSize[0], yt))

        # 上下周期性(靠近Y边界的纤维)
        if yt < fiberRadius:
            all_fiber_centers.append((xt, yt + rveSize[1]))
        if yt > rveSize[1] - fiberRadius:
            all_fiber_centers.append((xt, yt - rveSize[1]))

        # 对角周期性 - 只在纤维真正靠近边角顶点时创建镜像
        # 这样可以避免过多的重复镜像,提高效率

        # 左下角 (0, 0)
        if xt < fiberRadius and yt < fiberRadius:
            dist_to_corner = math.sqrt(xt ** 2 + yt ** 2)
            if dist_to_corner < fiberRadius:
                all_fiber_centers.append((xt + rveSize[0], yt + rveSize[1]))

        # 左上角 (0, height)
        if xt < fiberRadius and yt > rveSize[1] - fiberRadius:
            dist_to_corner = math.sqrt(xt ** 2 + (rveSize[1] - yt) ** 2)
            if dist_to_corner < fiberRadius:
                all_fiber_centers.append((xt + rveSize[0], yt - rveSize[1]))

        # 右上角 (width, height)
        if xt > rveSize[0] - fiberRadius and yt > rveSize[1] - fiberRadius:
            dist_to_corner = math.sqrt((rveSize[0] - xt) ** 2 + (rveSize[1] - yt) ** 2)
            if dist_to_corner < fiberRadius:
                all_fiber_centers.append((xt - rveSize[0], yt - rveSize[1]))

        # 右下角 (width, 0)
        if xt > rveSize[0] - fiberRadius and yt < fiberRadius:
            dist_to_corner = math.sqrt((rveSize[0] - xt) ** 2 + yt ** 2)
            if dist_to_corner < fiberRadius:
                all_fiber_centers.append((xt - rveSize[0], yt + rveSize[1]))

    # 去除重复点(可能在边角处产生)
    unique_centers = []
    tolerance = 1e-6

    for center in all_fiber_centers:
        is_duplicate = False
        for existing in unique_centers:
            if abs(center[0] - existing[0]) < tolerance and abs(center[1] - existing[1]) < tolerance:
                is_duplicate = True
                break
        if not is_duplicate:
            unique_centers.append(center)

    return unique_centers


def getCellCenterFromVertices(cell):
    """通过体单元的顶点坐标计算几何中心

    参数:
        cell: Abaqus体单元对象

    返回:
        (x, y, z): 体单元的几何中心坐标,如果失败则返回None

    功能:
        通过遍历体单元的所有顶点,计算其平均坐标作为几何中心
        这是getCentroid()方法的备用方案
    """
    try:
        vertices = cell.getVertices()
        if not vertices or len(vertices) == 0:
            return None

        x_coords = []
        y_coords = []
        z_coords = []

        for vertex in vertices:
            try:
                if hasattr(vertex, 'pointOn'):
                    coord = vertex.pointOn[0]
                    x_coords.append(coord[0])
                    y_coords.append(coord[1])
                    z_coords.append(coord[2])
            except:
                continue

        if len(x_coords) > 0 and len(y_coords) > 0 and len(z_coords) > 0:
            center_x = sum(x_coords) / len(x_coords)
            center_y = sum(y_coords) / len(y_coords)
            center_z = sum(z_coords) / len(z_coords)
            return (center_x, center_y, center_z)
        else:
            return None

    except Exception as e:
        print("      Debug: Error in getCellCenterFromVertices: %s" % str(e))
        return None


def classifyCellsImproved(all_cells, fiber_centers, rveSize, fiberRadius, rveVolume):
    """改进的体单元分类算法:利用纤维位置信息准确识别纤维和基体

    核心思想:
    1. 首先按体积排序,最大的体一定是主基体
    2. 构建包含周期性镜像的完整纤维中心列表
    3. 对于其他体,计算其质心到最近纤维中心的距离(仅XY平面)
    4. 如果距离 < 纤维半径,则该体是纤维;否则是基体碎片

    与2D版本的主要区别:
    - 2D处理面(face),3D处理体(cell)
    - 距离计算只考虑XY平面(因为纤维沿Z方向连续)
    - 体积验证使用圆柱体体积公式

    参数:
        all_cells: 所有体单元的列表
        fiber_centers: 原始RVE内的纤维中心 [(x1,y1), ...]
        rveSize: RVE尺寸 [width, height, depth]
        fiberRadius: 纤维半径
        rveVolume: RVE总体积

    返回:
        fiber_cells_list: 纤维体列表
        matrix_cells_list: 基体体列表(包括主基体和碎片)
    """
    print("\n  === Starting Improved Cell Classification (3D) ===")
    print("  Total cells to classify: %d" % len(all_cells))
    print("  Original fiber count: %d" % len(fiber_centers))

    # 步骤1: 按体积排序,最大的一定是主基体
    sorted_cells = sorted(all_cells, key=lambda c: c.getSize(), reverse=True)
    print("\n  Step 1: Volume-based sorting")
    print("    Largest cell volume: %.6e (assumed to be main matrix)" % sorted_cells[0].getSize())

    # 步骤2: 构建完整的纤维中心列表(包括周期性镜像)
    all_fiber_centers = buildAllFiberCenters3D(fiber_centers, rveSize, fiberRadius)
    print("\n  Step 2: Building complete fiber center list")
    print("    Original fibers: %d" % len(fiber_centers))
    print("    Total fiber centers (with XY periodicity): %d" % len(all_fiber_centers))

    # 步骤3: 初步分类
    matrix_cells_list = [sorted_cells[0]]  # 最大的体一定是主基体
    potential_cells = sorted_cells[1:]  # 其他体需要进一步判断

    print("\n  Step 3: Initial classification")
    print("    Main matrix cell: 1")
    print("    Cells to classify: %d" % len(potential_cells))

    # 步骤4: 详细分类 - 使用几何判断
    print("\n  Step 4: Detailed classification using geometry")

    # 检查数量关系,判断是否需要逐个识别
    if len(potential_cells) < len(all_fiber_centers):
        # 异常情况:体数量少于纤维数量(包含镜像)
        error_msg = (
                "FATAL ERROR: Cell count anomaly detected!\n"
                " -> Potential cells: %d\n"
                " -> Total fiber centers (with periodicity): %d\n"
                " -> This should not happen in a properly generated RVE."
                % (len(potential_cells), len(all_fiber_centers))
        )
        raise Exception(error_msg)

    elif len(potential_cells) == len(all_fiber_centers):
        # 理想情况:体数量等于纤维数量(包含镜像)
        print("    Potential cell count equals total fiber centers (%d = %d)" %
              (len(potential_cells), len(all_fiber_centers)))
        print("    Perfect match - all potential cells are fibers")
        fiber_cells_list = potential_cells
        matrix_fragment_count = 0

    else:
        # 一般情况:体数量 > 纤维数量,存在基体碎片
        print("    Potential cell count > total fiber centers (%d > %d)" %
              (len(potential_cells), len(all_fiber_centers)))
        print("    Matrix fragments detected - performing geometry-based classification")

        fiber_cells_list = []
        matrix_fragment_count = 0

        for idx, cell in enumerate(potential_cells):
            # 获取体的质心(使用多种方法确保稳定性)
            cell_center = None

            # 方法1: 使用getCentroid()
            try:
                cell_centroid = cell.getCentroid()
                if cell_centroid and len(cell_centroid) >= 3:
                    cell_center = (cell_centroid[0], cell_centroid[1], cell_centroid[2])
            except:
                pass

            # 方法2: 如果方法1失败,使用顶点平均
            if cell_center is None:
                cell_center = getCellCenterFromVertices(cell)

            # 如果仍然失败,报错
            if cell_center is None:
                error_msg = (
                        "FATAL ERROR: Unable to determine cell center for cell %d\n"
                        " -> All center calculation methods failed\n"
                        " -> Cell volume: %.6e\n"
                        " -> Cannot proceed with classification"
                        % (idx, cell.getSize())
                )
                raise Exception(error_msg)

            cell_x, cell_y, cell_z = cell_center[0], cell_center[1], cell_center[2]

            # 计算到所有纤维中心的最小距离(仅考虑XY平面)
            min_dist = float('inf')
            for fc_x, fc_y in all_fiber_centers:
                # 只计算XY平面的距离,忽略Z方向(因为纤维沿Z方向连续)
                dist = math.sqrt((cell_x - fc_x) ** 2 + (cell_y - fc_y) ** 2)
                if dist < min_dist:
                    min_dist = dist

            # 判断:如果最小距离 < 纤维半径,则是纤维体;否则是基体碎片
            if min_dist < fiberRadius:
                fiber_cells_list.append(cell)
            else:
                matrix_cells_list.append(cell)
                matrix_fragment_count += 1
                print("      [Fragment %d] Volume=%.2e, XY-CenterDist=%.6f > Radius=%.6f" %
                      (matrix_fragment_count, cell.getSize(), min_dist, fiberRadius))

    # 步骤5: 输出分类结果
    print("\n  === Classification Complete ===")
    print("  Fiber cells identified: %d" % len(fiber_cells_list))
    print("  Matrix cells total: %d" % len(matrix_cells_list))
    print("    - Main matrix: 1")
    if len(potential_cells) > len(all_fiber_centers):
        print("    - Fragments: %d" % (len(matrix_cells_list) - 1))

    # 步骤6: 验证体积分数
    fiber_total_volume = sum([c.getSize() for c in fiber_cells_list])
    actual_Vf = fiber_total_volume / rveVolume

    # 理论体积分数(基于纤维数量)
    # 三维: Vf = (N × π × R² × L) / (W × H × L) = (N × π × R²) / (W × H)
    target_Vf_from_count = (len(fiber_centers) * math.pi * fiberRadius ** 2) / (rveSize[0] * rveSize[1])

    print("\n  Validation:")
    print("    Target Vf (from fiber count): %.4f" % target_Vf_from_count)
    print("    Actual Vf (from classified cells): %.4f" % actual_Vf)

    deviation = abs(actual_Vf - target_Vf_from_count) / target_Vf_from_count * 100
    print("    Deviation: %.2f%%" % deviation)

    # 如果偏差较大,给出警告
    if deviation > 5.0:
        print("    WARNING: Actual Vf deviates >5%% from target!")
        print("             This may indicate issues with geometry or classification.")

    # 输出纤维体积的统计信息
    if fiber_cells_list:
        fiber_volumes = [c.getSize() for c in fiber_cells_list]
        print("    Fiber cell volumes: min=%.6e, max=%.6e, avg=%.6e" %
              (min(fiber_volumes), max(fiber_volumes), sum(fiber_volumes) / len(fiber_volumes)))

    return fiber_cells_list, matrix_cells_list


# =================================================================
#                 主建模函数
# =================================================================
def create3DRVEModel(modelName='Model-1',
                     rveSize=[0.057, 0.057, 0.01],
                     fiberRadius=0.0035,
                     target_Vf=0.5,
                     minDistanceFactor=2.05,
                     globalSeedSize=0.001,
                     deviationFactor=0.1,
                     minSizeFactor=0.1,
                     pairingToleranceFactor=0.5,
                     rsa_seeding_ratio=0.9,
                     export_coordinates=True,
                     csv_filename=None,
                     # 纤维材料参数
                     fiber_E1=230.0,
                     fiber_E2=15.0,
                     fiber_G12=15.0,
                     fiber_G23=7.0,
                     fiber_nu12=0.2,
                     # 基体材料参数
                     matrix_E=3170.0,
                     matrix_nu=0.35,
                     matrix_friction_angle=16.0,
                     matrix_flow_stress_ratio=1.0,
                     matrix_dilation_angle=16.0,
                     matrix_hardening_yield=106.4,
                     matrix_hardening_plastic_strain=0.0,
                     matrix_damage_strain=0.01,
                     matrix_damage_stress_triax=0.0,
                     matrix_damage_strain_rate=0.0,
                     matrix_damage_displacement=5e-05,
                     # 界面材料参数
                     cohesive_K_nn=1e8,
                     cohesive_K_ss=1e8,
                     cohesive_K_tt=1e8,
                     cohesive_t_n=44.0,
                     cohesive_t_s=82.0,
                     cohesive_t_t=82.0,
                     cohesive_GIC=0.001,
                     cohesive_GIIC=0.002,
                     cohesive_GIIIC=0.002,
                     cohesive_eta=1.5,
                     cohesive_stab_coeff=0.0001):
    """创建三维RVE模型的主函数

    参数说明:
        modelName: 模型名称
        rveSize: RVE尺寸 [宽度X, 高度Y, 深度Z](纤维沿Z方向)
        fiberRadius: 纤维半径
        target_Vf: 目标体积分数
        minDistanceFactor: 最小间距因子(实际最小间距 = 因子 × 纤维半径)
        globalSeedSize: 全局网格尺寸
        deviationFactor: 网格偏差因子
        minSizeFactor: 最小网格尺寸因子
        pairingToleranceFactor: 边界节点配对容差因子
        rsa_seeding_ratio: RSA播种比例(0-1,值越大分布越均匀)
        export_coordinates: 是否导出坐标到CSV文件
        csv_filename: CSV文件名(None则自动生成)

        纤维材料参数 (GPa) - 正交各向异性:
        fiber_E1: 纤维轴向模量(Direction-1,沿Z轴)
        fiber_E2: 纤维横向模量(Direction-2和Direction-3)
        fiber_G12: 纤维纵向剪切模量
        fiber_G23: 纤维横向剪切模量
        fiber_nu12: 纤维主泊松比

        基体材料参数 (MPa):
        matrix_E: 基体弹性模量
        matrix_nu: 基体泊松比
        matrix_friction_angle: Drucker-Prager摩擦角
        matrix_flow_stress_ratio: Drucker-Prager流动应力比
        matrix_dilation_angle: Drucker-Prager膨胀角
        matrix_hardening_yield: 硬化屈服应力
        matrix_hardening_plastic_strain: 硬化对应塑性应变
        matrix_damage_strain: 韧性损伤起始应变
        matrix_damage_stress_triax: 应力三轴度
        matrix_damage_strain_rate: 应变率
        matrix_damage_displacement: 损伤演化位移

        界面材料参数:
        cohesive_K_nn, cohesive_K_ss, cohesive_K_tt: 界面刚度 (N/mm^3)
        cohesive_t_n, cohesive_t_s, cohesive_t_t: 界面强度 (MPa)
        cohesive_GIC, cohesive_GIIC, cohesive_GIIIC: 断裂能 (N/mm)
        cohesive_eta: BK准则指数
        cohesive_stab_coeff: 粘性稳定系数
    """

    print("\n" + "=" * 70)
    print("Starting 3D RVE Model Generation")
    print("=" * 70)

    # ==================== 步骤 1: 参数计算和坐标生成 ====================
    print("\nStep 1: Calculating parameters and generating fiber coordinates...")

    depth = rveSize[2]
    rveArea = rveSize[0] * rveSize[1]
    fiberArea = math.pi * fiberRadius ** 2
    fiberCount = int(round((target_Vf * rveArea) / fiberArea))
    minDistance = minDistanceFactor * fiberRadius
    rsa_max_attempts = 500

    print("Target Vf: %.4f" % target_Vf)
    print("Calculated Fiber Count: %d" % fiberCount)
    print("RVE Size: %.6f x %.6f x %.6f" % (rveSize[0], rveSize[1], depth))
    print("Fiber Radius: %.6f, Length: %.6f" % (fiberRadius, depth))
    print("Min Distance: %.6f (%.2f x radius)" % (minDistance, minDistanceFactor))
    print("RSA Seeding Ratio: %.2f" % rsa_seeding_ratio)

    if fiberCount == 0:
        print("Warning: Fiber count is zero.")
        fiber_centers = []
    elif fiberArea >= rveArea:
        print("Error: Single fiber area >= RVE area. Aborting.")
        return
    else:
        print("\n--- Stage 1: RSA Seeding ---")
        seeding_count = int(fiberCount * rsa_seeding_ratio)
        seeded_coords = []

        for i in range(seeding_count):
            placed = False
            for _ in range(rsa_max_attempts):
                xt = rd.uniform(0, rveSize[0])
                yt = rd.uniform(0, rveSize[1])
                is_too_close = False

                for xc, yc in seeded_coords:
                    dx = abs(xt - xc)
                    dy = abs(yt - yc)
                    if dx > rveSize[0] / 2: dx = rveSize[0] - dx
                    if dy > rveSize[1] / 2: dy = rveSize[1] - dy

                    if dx * dx + dy * dy < minDistance ** 2:
                        is_too_close = True
                        break

                if not is_too_close:
                    seeded_coords.append((xt, yt))
                    placed = True
                    break

            if not placed:
                print("RSA seeding congested at %d fibers." % len(seeded_coords))
                break

        seeding_count_actual = len(seeded_coords)
        print("RSA placed %d anchor fibers." % seeding_count_actual)

        print("\n--- Stage 2: Random Placement ---")
        remaining = fiberCount - seeding_count_actual
        print("Placing %d fluid fibers..." % remaining)
        initial_coords = list(seeded_coords)

        for _ in range(remaining):
            initial_coords.append((rd.uniform(0, rveSize[0]),
                                   rd.uniform(0, rveSize[1])))

        print("\n--- Stage 3: Anchored Relaxation ---")
        relaxed_coords = _relax_coords_anchored(
            initial_coords, seeding_count_actual, fiberCount,
            rveSize, fiberRadius, minDistance
        )

        print("\n--- Stage 4: Final Verification ---")
        try:
            fiber_centers = _final_check_and_enforce(
                relaxed_coords, fiberCount, rveSize, minDistance
            )
        except Exception as e:
            print("\n" + "#" * 70)
            print(str(e))
            print("#" * 70 + "\n")
            return

    # 导出坐标
    if export_coordinates and fiber_centers:
        if csv_filename is None:
            csv_filename = "FiberCenters_3D_Vf%d_%s.csv" % (
                int(target_Vf * 100), time.strftime("%Y%m%d_%H%M%S")
            )
        exportFiberCentersToCSV(fiber_centers, csv_filename,
                                rveSize, fiberRadius, depth, target_Vf)

    # 验证纤维间距
    if fiber_centers:
        verification_passed, verification_stats = verifyMinimumFiberDistance3D(
            fiber_centers, rveSize, fiberRadius, minDistanceFactor
        )

        if not verification_passed:
            print("\nWARNING: Distance verification failed!")
            raise Exception("Minimum distance requirement not satisfied!")

    # ==================== 步骤 2: 创建周期性镜像 ====================
    print("\nStep 2: Creating periodic mirror coordinates...")

    xCoords, yCoords = [], []

    for xt, yt in fiber_centers:
        points_to_add = [(xt, yt)]

        if xt < fiberRadius:
            points_to_add.append((xt + rveSize[0], yt))
        if xt > rveSize[0] - fiberRadius:
            points_to_add.append((xt - rveSize[0], yt))
        if yt < fiberRadius:
            points_to_add.append((xt, yt + rveSize[1]))
        if yt > rveSize[1] - fiberRadius:
            points_to_add.append((xt, yt - rveSize[1]))

        if xt < fiberRadius and yt > rveSize[1] - fiberRadius:
            points_to_add.append((xt + rveSize[0], yt - rveSize[1]))
        if xt < fiberRadius and yt < fiberRadius:
            points_to_add.append((xt + rveSize[0], yt + rveSize[1]))
        if xt > rveSize[0] - fiberRadius and yt > rveSize[1] - fiberRadius:
            points_to_add.append((xt - rveSize[0], yt - rveSize[1]))
        if xt > rveSize[0] - fiberRadius and yt < fiberRadius:
            points_to_add.append((xt - rveSize[0], yt + rveSize[1]))

        unique_points = sorted(list(set(points_to_add)))
        for p in unique_points:
            xCoords.append(p[0])
            yCoords.append(p[1])

    print("Total coordinates (including mirrors): %d" % len(xCoords))

    # ==================== 步骤 3: 创建几何 ====================
    print("\nStep 3: Creating Abaqus 3D geometry...")

    if modelName in mdb.models:
        del mdb.models[modelName]

    mdb.Model(name=modelName, modelType=STANDARD_EXPLICIT)
    model = mdb.models[modelName]

    if 'Model-1' in mdb.models and modelName != 'Model-1':
        if len(mdb.models['Model-1'].parts) == 0:
            del mdb.models['Model-1']

    print("Creating matrix part...")
    s_matrix = model.ConstrainedSketch(name='MatrixSketch',
                                       sheetSize=max(rveSize) * 2)
    s_matrix.rectangle(point1=(0.0, 0.0),
                       point2=(rveSize[0], rveSize[1]))

    p_matrix = model.Part(name='Matrix',
                          dimensionality=THREE_D,
                          type=DEFORMABLE_BODY)
    p_matrix.BaseSolidExtrude(sketch=s_matrix, depth=depth)

    if fiberCount > 0:
        print("Creating fiber part with %d fibers..." % len(xCoords))
        s_fiber = model.ConstrainedSketch(name='FiberSketch',
                                          sheetSize=max(rveSize) * 3)

        for i in range(len(xCoords)):
            s_fiber.CircleByCenterPerimeter(
                center=(xCoords[i], yCoords[i]),
                point1=(xCoords[i] + fiberRadius, yCoords[i])
            )

        p_fiber = model.Part(name='Fiber',
                             dimensionality=THREE_D,
                             type=DEFORMABLE_BODY)
        p_fiber.BaseSolidExtrude(sketch=s_fiber, depth=depth)

        print("Fiber part created successfully.")

    # ==================== 步骤 4: 合并 ====================
    print("\nStep 4: Creating assembly and merging parts...")

    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)

    inst_matrix = assembly.Instance(name='Matrix-1',
                                    part=p_matrix,
                                    dependent=OFF)

    if fiberCount > 0:
        inst_fiber = assembly.Instance(name='Fiber-1',
                                       part=p_fiber,
                                       dependent=OFF)

        print("Performing Boolean merge...")
        assembly.InstanceFromBooleanMerge(
            name='RVE-3D',
            instances=(inst_matrix, inst_fiber),
            keepIntersections=ON,
            originalInstances=SUPPRESS,
            domain=GEOMETRY
        )
        print("Boolean merge completed.")
    else:
        assembly.features.changeKey(fromName='Matrix-1',
                                    toName='RVE-3D-1')

    # ==================== 步骤 5: 裁剪 ====================
    print("\nStep 5: Trimming to RVE boundaries...")

    p_rve = model.parts['RVE-3D']

    faces = p_rve.faces
    edges = p_rve.edges

    # 找到顶面
    top_face = None
    for face in faces:
        bbox = face.pointOn[0]
        if abs(bbox[2] - depth) < 1e-6:
            top_face = face
            break

    if top_face is None:
        top_face = faces[0]

    top_edge = edges[0]

    t = p_rve.MakeSketchTransform(
        sketchPlane=top_face,
        sketchUpEdge=top_edge,
        sketchPlaneSide=SIDE1,
        sketchOrientation=RIGHT,
        origin=(rveSize[0] / 2, rveSize[1] / 2, depth)
    )

    s_cut = model.ConstrainedSketch(name='CutSketch',
                                    sheetSize=max(rveSize) * 3,
                                    transform=t)

    p_rve.projectReferencesOntoSketch(sketch=s_cut,
                                      filter=COPLANAR_EDGES)

    margin = 2 * fiberRadius
    s_cut.rectangle(point1=(-rveSize[0] / 2, -rveSize[1] / 2),
                    point2=(rveSize[0] / 2, rveSize[1] / 2))
    s_cut.rectangle(point1=(-rveSize[0] / 2 - margin, -rveSize[1] / 2 - margin),
                    point2=(rveSize[0] / 2 + margin, rveSize[1] / 2 + margin))

    p_rve.CutExtrude(
        sketchPlane=top_face,
        sketchUpEdge=top_edge,
        sketchPlaneSide=SIDE1,
        sketchOrientation=RIGHT,
        sketch=s_cut,
        flipExtrudeDirection=OFF
    )

    print("Trimming completed.")

    # ==================== 步骤 6: 创建集合 ====================
    print("\nStep 6: Creating geometry sets...")

    p_rve = model.parts['RVE-3D']
    all_cells = p_rve.cells
    p_rve.Set(cells=all_cells, name='set_AllCells')

    rveVolume = rveSize[0] * rveSize[1] * depth

    if len(all_cells) > 1 and fiber_centers:
        # 使用改进的分类算法
        fiber_cells_list, matrix_cells_list = classifyCellsImproved(
            all_cells, fiber_centers, rveSize, fiberRadius, rveVolume
        )

        if matrix_cells_list:
            p_rve.Set(name='set_MatrixCell', cells=CellArray(matrix_cells_list))
        else:
            p_rve.Set(name='set_MatrixCell', cells=CellArray())

        if fiber_cells_list:
            p_rve.Set(name='set_FiberCells', cells=CellArray(fiber_cells_list))
        else:
            p_rve.Set(name='set_FiberCells', cells=CellArray())
    else:
        p_rve.Set(name='set_MatrixCell',
                  cells=all_cells if len(all_cells) == 1 else CellArray())
        p_rve.Set(name='set_FiberCells', cells=CellArray())

    all_faces = p_rve.faces
    p_rve.Set(faces=all_faces, name='set_AllFaces')

    tol = 1e-6
    face_left = all_faces.getByBoundingBox(-tol, -tol, -tol, tol, rveSize[1] + tol, depth + tol)
    face_right = all_faces.getByBoundingBox(rveSize[0] - tol, -tol, -tol,
                                            rveSize[0] + tol, rveSize[1] + tol, depth + tol)
    face_front = all_faces.getByBoundingBox(-tol, -tol, -tol, rveSize[0] + tol, tol, depth + tol)
    face_back = all_faces.getByBoundingBox(-tol, rveSize[1] - tol, -tol,
                                           rveSize[0] + tol, rveSize[1] + tol, depth + tol)
    face_bottom = all_faces.getByBoundingBox(-tol, -tol, -tol, rveSize[0] + tol, rveSize[1] + tol, tol)
    face_top = all_faces.getByBoundingBox(-tol, -tol, depth - tol,
                                          rveSize[0] + tol, rveSize[1] + tol, depth + tol)

    outer_faces = face_left + face_right + face_front + face_back + face_bottom + face_top
    p_rve.Set(faces=outer_faces, name='set_OuterFaces')

    p_rve.SetByBoolean(name='set_CohesiveFaces',
                       sets=(p_rve.sets['set_AllFaces'], p_rve.sets['set_OuterFaces']),
                       operation=DIFFERENCE)

    print("  Cohesive faces: %d" % len(p_rve.sets['set_CohesiveFaces'].faces))
    print("Step 6 Complete.")

    # ==================== 步骤 7: 材料和截面 ====================
    print("\nStep 7: Defining materials and sections...")

    # 基体材料
    matrixMaterial = model.Material(name='Material-Matrix')
    matrixMaterial.Elastic(table=((matrix_E, matrix_nu),))
    matrixMaterial.DruckerPrager(table=((matrix_friction_angle,
                                         matrix_flow_stress_ratio,
                                         matrix_dilation_angle),))
    matrixMaterial.druckerPrager.DruckerPragerHardening(
        table=((matrix_hardening_yield, matrix_hardening_plastic_strain),))
    matrixMaterial.DuctileDamageInitiation(
        table=((matrix_damage_strain,
                matrix_damage_stress_triax,
                matrix_damage_strain_rate),))
    matrixMaterial.ductileDamageInitiation.DamageEvolution(
        type=DISPLACEMENT, table=((matrix_damage_displacement,),))

    # 纤维材料(正交各向异性)
    fiberMaterial = model.Material(name='Material-Fiber')
    fiberMaterial.Elastic(
        type=ENGINEERING_CONSTANTS,
        table=((fiber_E1 * 1000,  # MPa转换
                fiber_E2 * 1000,
                fiber_E2 * 1000,
                fiber_nu12,
                fiber_nu12,
                0.25,
                fiber_G12 * 1000,
                fiber_G12 * 1000,
                fiber_G23 * 1000),))

    # 界面材料
    cohesiveMaterial = model.Material(name='Material-Cohesive')
    cohesiveMaterial.Elastic(type=TRACTION,
                             table=((cohesive_K_nn, cohesive_K_ss, cohesive_K_tt),))
    cohesiveMaterial.QuadsDamageInitiation(
        table=((cohesive_t_n, cohesive_t_s, cohesive_t_t),))
    cohesiveMaterial.quadsDamageInitiation.DamageEvolution(
        type=ENERGY, mixedModeBehavior=BK, power=cohesive_eta,
        table=((cohesive_GIC, cohesive_GIIC, cohesive_GIIIC),))
    cohesiveMaterial.quadsDamageInitiation.DamageStabilizationCohesive(
        cohesiveCoeff=cohesive_stab_coeff)

    model.HomogeneousSolidSection(name='Section-Fiber',
                                  material='Material-Fiber', thickness=None)
    model.HomogeneousSolidSection(name='Section-Matrix',
                                  material='Material-Matrix', thickness=None)
    model.CohesiveSection(name='Section-Cohesive',
                          material='Material-Cohesive',
                          response=TRACTION_SEPARATION,
                          outOfPlaneThickness=None)

    if 'set_FiberCells' in p_rve.sets and p_rve.sets['set_FiberCells'].cells:
        p_rve.SectionAssignment(region=p_rve.sets['set_FiberCells'],
                                sectionName='Section-Fiber', offset=0.0,
                                offsetType=MIDDLE_SURFACE, offsetField='')

    if 'set_MatrixCell' in p_rve.sets and p_rve.sets['set_MatrixCell'].cells:
        p_rve.SectionAssignment(region=p_rve.sets['set_MatrixCell'],
                                sectionName='Section-Matrix', offset=0.0,
                                offsetType=MIDDLE_SURFACE, offsetField='')

    # 纤维材料方向指派
    if 'set_FiberCells' in p_rve.sets and len(p_rve.sets['set_FiberCells'].cells) > 0:
        print("\n  Assigning fiber material orientation...")
        try:
            region_fiber = p_rve.sets['set_FiberCells']

            # 将材料Direction-1对齐到全局Z轴
            p_rve.MaterialOrientation(
                region=region_fiber,
                orientationType=SYSTEM,
                axis=AXIS_3,  # 全局Z轴
                localCsys=None,
                additionalRotationType=ROTATION_NONE,
                stackDirection=STACK_1  # Direction-1对齐
            )
            print("  SUCCESS: Fiber orientation assigned - Direction-1 along Z-axis (+Z)")

        except Exception as e1:
            print("  WARNING: SYSTEM method failed: %s" % str(e1))
            print("  Attempting alternative method...")

            try:
                # 方法2: 创建局部坐标系
                origin_pt = p_rve.InterestingPoint(
                    p_rve.edges[0],
                    CENTER
                )
                datum_csys = p_rve.DatumCsysByThreePoints(
                    origin=origin_pt,
                    point1=(origin_pt[0] + 1.0, origin_pt[1], origin_pt[2]),
                    point2=(origin_pt[0], origin_pt[1] + 1.0, origin_pt[2]),
                    name='Datum-FiberOrientation',
                    coordSysType=CARTESIAN
                )

                p_rve.MaterialOrientation(
                    region=region_fiber,
                    orientationType=SYSTEM,
                    axis=AXIS_3,
                    localCsys=datum_csys,
                    additionalRotationType=ROTATION_NONE,
                    stackDirection=STACK_1
                )
                print("  SUCCESS: Fiber orientation assigned using local coordinate system")

            except Exception as e2:
                print("  ERROR: Alternative method also failed: %s" % str(e2))
                print("  WARNING: Fiber material orientation NOT assigned!")
                print("  Please assign orientation manually in Abaqus/CAE:")
                print("    Property -> Material Orientation -> Edit")
                print("    Set Axis: AXIS_3, Stack Direction: STACK_1")

    print("Step 7 Complete.")

    # ==================== 步骤 8: 网格 ====================
    print("\nStep 8: Meshing RVE...")
    print("  Global seed size: %.6f" % globalSeedSize)
    print("  Deviation factor: %.2f" % deviationFactor)
    print("  Min size factor: %.2f" % minSizeFactor)

    p_rve.seedPart(size=globalSeedSize, deviationFactor=deviationFactor,
                   minSizeFactor=minSizeFactor, constraint=FREE)

    elemType_bulk = ElemType(elemCode=C3D8R, elemLibrary=STANDARD,
                             kinematicSplit=AVERAGE_STRAIN,
                             secondOrderAccuracy=OFF,
                             hourglassControl=DEFAULT,
                             distortionControl=DEFAULT,
                             elemDeletion=ON, maxDegradation=0.99)

    elemType_bulk_wedge = ElemType(elemCode=C3D6, elemLibrary=STANDARD)
    elemType_bulk_tet = ElemType(elemCode=C3D4, elemLibrary=STANDARD)

    p_rve.setElementType(regions=(p_rve.sets['set_AllCells'].cells,),
                         elemTypes=(elemType_bulk, elemType_bulk_wedge, elemType_bulk_tet))

    print("  Generating mesh...")
    p_rve.generateMesh()

    total_elems = len(p_rve.elements)
    print("  Total elements generated: %d" % total_elems)

    if total_elems == 0:
        print("  ERROR: No mesh generated!")
        return

    print("Step 8 Complete.")

    # ==================== 步骤 9: 粘接单元 ====================
    print("\nStep 9: Inserting cohesive elements...")

    if 'set_CohesiveFaces' in p_rve.sets and len(p_rve.sets['set_CohesiveFaces'].faces) > 0:
        p_rve.insertElements(faces=p_rve.sets['set_CohesiveFaces'])

        all_elements = p_rve.elements
        p_rve.Set(elements=all_elements, name='set_AllElements')

        if len(p_rve.sets['set_FiberCells'].cells) > 0:
            p_rve.Set(elements=p_rve.sets['set_FiberCells'].elements,
                      name='set_FiberElements')
        else:
            p_rve.Set(elements=ElementArray([]), name='set_FiberElements')

        if len(p_rve.sets['set_MatrixCell'].cells) > 0:
            p_rve.Set(elements=p_rve.sets['set_MatrixCell'].elements,
                      name='set_MatrixElements')
        else:
            p_rve.Set(elements=ElementArray([]), name='set_MatrixElements')

        p_rve.SetByBoolean(name='set_CohesiveElements',
                           sets=(p_rve.sets['set_AllElements'],
                                 p_rve.sets['set_FiberElements'],
                                 p_rve.sets['set_MatrixElements']),
                           operation=DIFFERENCE)

        print("  Total elements: %d" % len(all_elements))
        print("  Fiber elements: %d" % len(p_rve.sets['set_FiberElements'].elements))
        print("  Matrix elements: %d" % len(p_rve.sets['set_MatrixElements'].elements))
        print("  Cohesive elements: %d" % len(p_rve.sets['set_CohesiveElements'].elements))

        if len(p_rve.sets['set_CohesiveElements'].elements) > 0:
            elemType_coh = ElemType(elemCode=COH3D8, elemLibrary=STANDARD,
                                    elemDeletion=ON, maxDegradation=0.99)

            p_rve.setElementType(regions=(p_rve.sets['set_CohesiveElements'].elements,),
                                 elemTypes=(elemType_coh,))

            p_rve.SectionAssignment(region=p_rve.sets['set_CohesiveElements'],
                                    sectionName='Section-Cohesive', offset=0.0)

            print("  Cohesive elements configured.")

    print("Step 9 Complete.")

    # ==================== 步骤 10: 周期性边界条件 ====================
    print("\nStep 10: Applying Periodic Boundary Conditions...")

    rootAssembly = model.rootAssembly
    rveInstance = rootAssembly.Instance(name='RVE-3D-1', part=p_rve, dependent=ON)

    createReferencePoints3D(model)

    dimensions = getRVEDimensions3D(model, 'RVE-3D-1')
    (nodes_left, nodes_right, nodes_front, nodes_back,
     nodes_bottom, nodes_top) = getBoundaryNodes3D(model, 'RVE-3D-1', dimensions)

    pairing_tolerance = globalSeedSize * pairingToleranceFactor
    print("  Global mesh size: %.6f" % globalSeedSize)
    print("  Min mesh size: %.6f" % (globalSeedSize * minSizeFactor))
    print("  PBC pairing tolerance: %.6f (%.1fx global mesh size)" %
          (pairing_tolerance, pairingToleranceFactor))

    if len(nodes_left) <= len(nodes_right):
        lr_pairs = pairBoundaryNodes3D(nodes_left, nodes_right,
                                       pairing_tolerance, (1, 2))
    else:
        lr_pairs = [(p[1], p[0]) for p in pairBoundaryNodes3D(
            nodes_right, nodes_left, pairing_tolerance, (1, 2))]

    if len(nodes_front) <= len(nodes_back):
        fb_pairs = pairBoundaryNodes3D(nodes_front, nodes_back,
                                       pairing_tolerance, (0, 2))
    else:
        fb_pairs = [(p[1], p[0]) for p in pairBoundaryNodes3D(
            nodes_back, nodes_front, pairing_tolerance, (0, 2))]

    if len(nodes_bottom) <= len(nodes_top):
        bt_pairs = pairBoundaryNodes3D(nodes_bottom, nodes_top,
                                       pairing_tolerance, (0, 1))
    else:
        bt_pairs = [(p[1], p[0]) for p in pairBoundaryNodes3D(
            nodes_top, nodes_bottom, pairing_tolerance, (0, 1))]

    unpaired_left = len(nodes_left) - len(lr_pairs)
    unpaired_right = len(nodes_right) - len(lr_pairs)
    unpaired_front = len(nodes_front) - len(fb_pairs)
    unpaired_back = len(nodes_back) - len(fb_pairs)
    unpaired_bottom = len(nodes_bottom) - len(bt_pairs)
    unpaired_top = len(nodes_top) - len(bt_pairs)

    print("\n  Boundary Node Pairing Results:")
    print("    Left/Right (X): %d pairs from %d/%d nodes" %
          (len(lr_pairs), len(nodes_left), len(nodes_right)))
    print("    Front/Back (Y): %d pairs from %d/%d nodes" %
          (len(fb_pairs), len(nodes_front), len(nodes_back)))
    print("    Bottom/Top (Z): %d pairs from %d/%d nodes" %
          (len(bt_pairs), len(nodes_bottom), len(nodes_top)))

    total_unpaired_x = unpaired_left + unpaired_right
    total_unpaired_y = unpaired_front + unpaired_back
    total_unpaired_z = unpaired_bottom + unpaired_top

    if total_unpaired_x > 0:
        print("    WARNING: %d nodes unpaired in X (Left:%d + Right:%d)" %
              (total_unpaired_x, unpaired_left, unpaired_right))
    if total_unpaired_y > 0:
        print("    WARNING: %d nodes unpaired in Y (Front:%d + Back:%d)" %
              (total_unpaired_y, unpaired_front, unpaired_back))
    if total_unpaired_z > 0:
        print("    WARNING: %d nodes unpaired in Z (Bottom:%d + Top:%d)" %
              (total_unpaired_z, unpaired_bottom, unpaired_top))

    if total_unpaired_x == 0 and total_unpaired_y == 0 and total_unpaired_z == 0:
        print("    SUCCESS: All boundary nodes paired successfully!")

    applyPeriodicConstraints3D(model, 'RVE-3D-1', lr_pairs, 'Left-Right')
    applyPeriodicConstraints3D(model, 'RVE-3D-1', fb_pairs, 'Front-Back')
    applyPeriodicConstraints3D(model, 'RVE-3D-1', bt_pairs, 'Bottom-Top')

    print("Step 10 Complete.")

    # ==================== 步骤 11: 清理多余Part ====================
    print("\nStep 11: Cleaning up temporary parts...")

    parts_to_delete = []
    if 'Fiber' in model.parts:
        parts_to_delete.append('Fiber')
    if 'Matrix' in model.parts:
        parts_to_delete.append('Matrix')

    for part_name in parts_to_delete:
        try:
            del model.parts[part_name]
            print("  Deleted part: %s" % part_name)
        except:
            print("  Could not delete part: %s" % part_name)

    print("Step 11 Complete.")

    # ==================== 完成 ====================
    print("\n" + "=" * 70)
    print("3D RVE MODEL GENERATION COMPLETED SUCCESSFULLY")
    print("=" * 70)
    print("\nModel Summary:")
    print("  Model Name: %s" % modelName)
    print("  RVE Dimensions: %.6f x %.6f x %.6f" %
          (rveSize[0], rveSize[1], depth))
    print("  Fiber Count: %d" % fiberCount)
    print("  Fiber Radius: %.6f" % fiberRadius)
    print("  Target Vf: %.4f" % target_Vf)
    print("  Fiber Orientation: Direction-1 along Z-axis (+Z)")
    print("  Periodic BCs: Applied with tolerance = %.2fx mesh size" % pairingToleranceFactor)
    print("  Cohesive Elements: Inserted at fiber-matrix interface")
    print("  Remaining Parts: %d" % len(model.parts))

    if total_unpaired_x > 0 or total_unpaired_y > 0 or total_unpaired_z > 0:
        print("\n  NOTE: Some boundary nodes could not be paired due to")
        print("        asymmetric mesh topology from fiber trimming.")
        print("        Consider increasing pairingToleranceFactor if needed.")


# =================================================================
#                 主程序入口
# =================================================================
if __name__ == '__main__':
    """主程序 - 设置参数并执行三维RVE建模"""

    # ========== 核心几何参数 ==========
    # TARGET_VF: 目标纤维体积分数
    # - 取值范围: 0.0 ~ 0.7 (理论上可达0.785,但实际受最小间距限制)
    # - 说明: 定义纤维在RVE中所占的体积比例,根据此值自动计算纤维数量
    # - 影响: 值越大,复合材料强度越高,但生成难度增加
    TARGET_VF = 0.75

    # RVE_SIZE: RVE模型的尺寸 [宽度X, 高度Y, 深度Z],单位mm
    # - 说明: 代表性体积单元的物理尺寸,纤维沿Z方向延伸
    # - 建议: XY尺寸应至少为纤维直径的10倍以上以保证统计代表性
    # - 影响: 尺寸越大,包含纤维数越多,计算成本越高,但统计性越好
    # - 注意: Z方向尺寸(深度)定义纤维长度
    RVE_SIZE = [0.057, 0.057, 0.01]

    # FIBER_RADIUS: 纤维半径,单位mm
    # - 说明: 单根纤维的半径,纤维直径 = 2 × 半径
    # - 影响: 决定了纤维的尺寸和数量(Vf相同时,半径越小纤维数越多)
    FIBER_RADIUS = 0.0035

    # ========== 约束参数 ==========
    # MIN_DIST_FACTOR: 最小间距因子
    # - 取值范围: 2.0 ~ 2.5 (推荐 2.0 ~ 2.1)
    # - 说明: 纤维中心之间的最小距离 = 因子 × 纤维半径
    #         例如:因子=2.05时,最小中心距=2.05R,纤维表面最小间隙=0.05R
    # - 影响:
    #   * 值越大: 纤维间隙越大,生成越容易,但可达到的最大Vf降低
    #   * 值越小: 可以达到更高的Vf,但生成难度大幅增加,可能失败
    # - 建议: 对于高Vf(>0.5),使用2.0~2.05;对于低Vf(<0.4),可用2.1~2.2
    MIN_DIST_FACTOR = 2.05

    # ========== 网格参数 ==========
    # GLOBAL_SEED_SIZE: 全局网格种子尺寸,单位mm
    # - 说明: 控制网格的精细程度,决定单元的平均边长
    # - 建议: 纤维半径的 1/7 ~ 1/10
    #         例如:半径0.0035mm时,种子尺寸0.0005mm (约1/7)
    # - 影响:
    #   * 值越小: 网格越精细,结果越准确,但计算成本显著增加
    #   * 值越大: 网格越粗糙,计算快但可能影响精度
    # - 注意: 纤维圆周至少需要20-30个单元以保证几何精度
    GLOBAL_SEED_SIZE = 0.0005

    # DEVIATION_FACTOR: 网格偏差因子
    # - 说明: 控制曲面网格偏离实际几何的最大允许偏差
    # - 取值范围: 0.01 ~ 0.5
    # - 影响: 值越小,网格越精细,越贴合曲面
    # - 建议: 0.1 (标准值)
    DEVIATION_FACTOR = 0.1

    # MIN_SIZE_FACTOR: 最小网格尺寸因子
    # - 说明: 最小单元尺寸 = 因子 × 全局网格尺寸
    # - 取值范围: 0.01 ~ 0.5
    # - 影响: 控制细节区域的最小网格尺寸
    # - 建议: 0.1 (允许局部细化到全局尺寸的1/10)
    MIN_SIZE_FACTOR = 0.1

    # ========== 周期性边界条件参数 ==========
    # PAIRING_TOLERANCE_FACTOR: 边界节点配对容差因子
    # - 说明: 周期性边界条件中,相对边界节点配对的容差
    #         实际容差 = 因子 × 全局网格尺寸
    # - 取值范围: 0.1 ~ 1.0 (推荐 0.5)
    # - 影响:
    #   * 容差太小: 可能导致配对失败,特别是在纤维截断边界处
    #   * 容差太大: 可能误配对不同位置的节点
    # - 建议: 0.5 (一般使用默认值即可,如遇配对问题可适当增大)
    PAIRING_TOLERANCE_FACTOR = 0.5

    # ========== 算法调优参数 ==========
    # RSA_SEEDING_RATIO: RSA播种比例
    # - 取值范围: 0.0 ~ 1.0
    # - 说明: 控制纤维分布模式的关键参数,决定有多少比例的纤维作为"锚点"
    # - 三种模式:
    #   * 高值 (0.8~1.0) - "均匀离散"模式:
    #     - 纤维均匀散布,避免团簇
    #     - 生成速度快(推荐用于生产环境)
    #     - 适合需要均匀分布的情况
    #   * 低值 (0.0~0.3) - "物理平衡"模式:
    #     - 模拟物理粒子排斥平衡
    #     - 会形成局部密集团簇(更接近真实材料)
    #     - 生成速度慢
    #   * 中等值 (0.4~0.7) - 混合模式:
    #     - 兼具两种模式的特点
    #     - 平衡速度和分布多样性
    # - 推荐: 0.9 (快速均匀分布) 或 0.1 (真实物理分布)
    RSA_SEEDING_RATIO = 0.9

    # ========== 纤维材料参数 (GPa) - 正交各向异性 ==========
    # FIBER_E1: 纤维轴向弹性模量,单位GPa
    # - 说明: 纤维沿轴向(Direction-1,即Z轴方向)的杨氏模量
    # - 典型值: 碳纤维 ~230 GPa, 玻璃纤维 ~70 GPa
    # - 注意: 这是纤维材料最重要的参数,决定了复合材料的轴向刚度
    FIBER_E1 = 230.0

    # FIBER_E2: 纤维横向弹性模量,单位GPa
    # - 说明: 纤维垂直于轴向(Direction-2和Direction-3)的杨氏模量
    # - 典型值: 通常远小于轴向模量,碳纤维 ~15 GPa
    # - 注意: E2 = E3 (横向各向同性假设)
    FIBER_E2 = 15.0

    # FIBER_G12: 纤维纵向剪切模量,单位GPa
    # - 说明: 纤维在1-2平面和1-3平面的剪切模量
    # - 典型值: 碳纤维 ~15 GPa
    # - 注意: G12 = G13 (对称性)
    FIBER_G12 = 15.0

    # FIBER_G23: 纤维横向剪切模量,单位GPa
    # - 说明: 纤维在2-3平面(横截面内)的剪切模量
    # - 典型值: 碳纤维 ~7 GPa
    # - 关系: 对于横向各向同性材料,G23 = E2 / (2*(1+nu23))
    FIBER_G23 = 7.0

    # FIBER_NU12: 纤维主泊松比
    # - 说明: 轴向拉伸时横向收缩应变与轴向应变的比值
    # - 取值范围: 0.0 ~ 0.5 (碳纤维典型值 0.2 ~ 0.3)
    # - 注意: nu12 = nu13 (对称性)
    FIBER_NU12 = 0.2

    # ========== 基体材料参数 (MPa) ==========
    # MATRIX_E: 基体弹性模量,单位MPa
    # - 说明: 基体材料的杨氏模量
    # - 典型值: 环氧树脂 ~3000 MPa, 聚酯 ~3500 MPa
    MATRIX_E = 3170.0

    # MATRIX_NU: 基体泊松比
    # - 说明: 基体材料的泊松比
    # - 典型值: 聚合物基体 0.3 ~ 0.4
    MATRIX_NU = 0.35

    # MATRIX_FRICTION_ANGLE: Drucker-Prager摩擦角,单位度
    # - 说明: 描述材料的剪切强度随压应力变化的参数
    # - 取值范围: 0° ~ 45° (典型值 10° ~ 30°)
    # - 影响: 角度越大,材料受压时强度增加越明显
    MATRIX_FRICTION_ANGLE = 16.0

    # MATRIX_FLOW_STRESS_RATIO: Drucker-Prager流动应力比 K
    # - 说明: 屈服面在偏平面上的形状参数,K=1.0表示圆形屈服面
    # - 取值范围: 0.778 ~ 1.0
    # - 影响: 控制拉压屈服强度的差异
    MATRIX_FLOW_STRESS_RATIO = 1.0

    # MATRIX_DILATION_ANGLE: Drucker-Prager膨胀角,单位度
    # - 说明: 控制塑性流动时的体积变化
    # - 取值范围: 0° ~ 摩擦角
    # - 影响: 角度越大,塑性变形时体积膨胀越明显
    # - 建议: 通常取与摩擦角相同或略小的值
    MATRIX_DILATION_ANGLE = 16.0

    # MATRIX_HARDENING_YIELD: 硬化屈服应力,单位MPa
    # - 说明: 材料开始塑性硬化时的屈服应力
    # - 影响: 定义了材料从弹性到塑性的转变点
    MATRIX_HARDENING_YIELD = 106.4

    # MATRIX_HARDENING_PLASTIC_STRAIN: 硬化对应的塑性应变
    # - 说明: 与硬化屈服应力对应的塑性应变值
    # - 影响: 配合屈服应力定义硬化曲线
    # - 注意: 0.0表示初始屈服点
    MATRIX_HARDENING_PLASTIC_STRAIN = 0.0

    # MATRIX_DAMAGE_STRAIN: 韧性损伤起始应变
    # - 说明: 材料开始发生韧性损伤时的等效塑性应变
    # - 影响: 值越小,材料越早开始损伤
    # - 典型值: 0.01 ~ 0.1
    MATRIX_DAMAGE_STRAIN = 0.01

    # MATRIX_DAMAGE_STRESS_TRIAX: 应力三轴度
    # - 说明: 静水压力与等效应力的比值,η = σm / σeq
    # - 取值范围: -1/3 (纯剪) 到 +∞ (静水拉伸)
    # - 影响: 描述应力状态对损伤的影响
    # - 注意: 0.0表示不考虑三轴度影响
    MATRIX_DAMAGE_STRESS_TRIAX = 0.0

    # MATRIX_DAMAGE_STRAIN_RATE: 应变率,单位1/s
    # - 说明: 损伤发生时的应变率
    # - 影响: 描述应变率对损伤的影响
    # - 注意: 0.0表示准静态加载
    MATRIX_DAMAGE_STRAIN_RATE = 0.0

    # MATRIX_DAMAGE_DISPLACEMENT: 损伤演化特征位移,单位mm
    # - 说明: 从损伤起始到完全失效的特征位移
    # - 影响: 值越大,材料损伤演化越慢,韧性越好
    # - 典型值: 0.00001 ~ 0.0001 mm
    # - 注意: 应根据网格尺寸调整,建议为单元尺寸的0.1~1倍
    MATRIX_DAMAGE_DISPLACEMENT = 5e-05

    # ========== 界面材料参数 ==========
    # COHESIVE_K_NN: 界面法向刚度,单位N/mm³
    # - 说明: 控制界面法向(垂直于界面)的初始刚度
    # - 建议: 取10⁷ ~ 10⁹,通常为基体模量的10³~10⁵倍
    # - 影响: 刚度越大,界面初始响应越硬
    COHESIVE_K_NN = 1e8

    # COHESIVE_K_SS: 界面第一切向刚度,单位N/mm³
    # - 说明: 控制界面第一个切向的初始刚度
    # - 建议: 通常与法向刚度相同或略小
    COHESIVE_K_SS = 1e8

    # COHESIVE_K_TT: 界面第二切向刚度,单位N/mm³
    # - 说明: 控制界面第二个切向的初始刚度
    # - 建议: 三维模型中通常与K_SS相同
    COHESIVE_K_TT = 1e8

    # COHESIVE_T_N: 界面法向强度,单位MPa
    # - 说明: 界面在法向(拉伸)方向的最大承载能力
    # - 影响: 决定界面开裂的起始载荷
    # - 典型值: 20 ~ 80 MPa
    COHESIVE_T_N = 44.0

    # COHESIVE_T_S: 界面第一切向强度,单位MPa
    # - 说明: 界面在第一个切向(剪切)方向的最大承载能力
    # - 影响: 决定界面剪切失效的起始载荷
    # - 典型值: 40 ~ 100 MPa (通常大于法向强度)
    COHESIVE_T_S = 82.0

    # COHESIVE_T_T: 界面第二切向强度,单位MPa
    # - 说明: 界面在第二个切向方向的最大承载能力
    # - 建议: 三维模型中通常与T_S相同
    COHESIVE_T_T = 82.0

    # COHESIVE_GIC: I型断裂能(法向),单位N/mm
    # - 说明: 界面完全分离所需的能量(拉伸模式)
    # - 影响: 决定界面韧性和损伤演化速度
    # - 典型值: 0.0001 ~ 0.01 N/mm
    # - 注意: 值越大,界面越韧,失效过程越渐进
    COHESIVE_GIC = 0.001

    # COHESIVE_GIIC: II型断裂能(切向),单位N/mm
    # - 说明: 界面完全分离所需的能量(剪切模式)
    # - 影响: 决定剪切失效的韧性
    # - 典型值: 通常为GIC的1~3倍
    COHESIVE_GIIC = 0.002

    # COHESIVE_GIIIC: III型断裂能(切向),单位N/mm
    # - 说明: 界面完全分离所需的能量(撕裂模式)
    # - 建议: 三维模型中通常与GIIC相同
    COHESIVE_GIIIC = 0.002

    # COHESIVE_ETA: BK准则指数
    # - 说明: Benzeggagh-Kenane混合模式断裂准则的指数
    # - 取值范围: 1.0 ~ 3.0
    # - 影响: 控制混合模式下断裂能的插值方式
    # - 建议: 1.5 ~ 2.0 (通过材料试验拟合得到)
    COHESIVE_ETA = 1.5

    # COHESIVE_STAB_COEFF: 粘性稳定系数
    # - 说明: 用于数值稳定性的人工阻尼系数
    # - 取值范围: 0.00001 ~ 0.001
    # - 影响:
    #   * 值太小: 可能导致数值不稳定
    #   * 值太大: 可能影响结果准确性
    # - 建议: 0.0001 (需要时可微调)
    COHESIVE_STAB_COEFF = 0.0001

    # ========== 输出控制 ==========
    # EXPORT_COORDINATES: 是否导出纤维中心坐标到CSV文件
    # - True: 导出坐标,便于后续分析和验证
    # - False: 不导出
    EXPORT_COORDINATES = True

    # CSV_FILENAME: CSV文件名
    # - None: 自动生成文件名(格式:FiberCenters_3D_Vf50_YYYYMMDD_HHMMSS.csv)
    # - 字符串: 使用指定的文件名
    CSV_FILENAME = None

    # ========== 模型命名 ==========
    targetModelName = '3D-RVE-Vf-%d' % (TARGET_VF * 100)
    tempModelName = targetModelName + '_TEMP_' + str(int(time.time()))

    print("\n" + "=" * 70)
    print("Starting 3D RVE Generation...")
    print("=" * 70)
    print("Target Model Name: %s" % targetModelName)
    print("PBC Tolerance Factor: %.2f (relative to global mesh size)" % PAIRING_TOLERANCE_FACTOR)

    if targetModelName in mdb.models:
        print("WARNING: Model exists and will be replaced after successful generation.")

    print("Using temporary name: %s" % tempModelName)
    print("=" * 70)

    # ========== 执行建模 ==========
    create3DRVEModel(
        modelName=tempModelName,
        rveSize=RVE_SIZE,
        fiberRadius=FIBER_RADIUS,
        target_Vf=TARGET_VF,
        minDistanceFactor=MIN_DIST_FACTOR,
        globalSeedSize=GLOBAL_SEED_SIZE,
        deviationFactor=DEVIATION_FACTOR,
        minSizeFactor=MIN_SIZE_FACTOR,
        pairingToleranceFactor=PAIRING_TOLERANCE_FACTOR,
        rsa_seeding_ratio=RSA_SEEDING_RATIO,
        export_coordinates=EXPORT_COORDINATES,
        csv_filename=CSV_FILENAME,
        # 纤维材料参数
        fiber_E1=FIBER_E1,
        fiber_E2=FIBER_E2,
        fiber_G12=FIBER_G12,
        fiber_G23=FIBER_G23,
        fiber_nu12=FIBER_NU12,
        # 基体材料参数
        matrix_E=MATRIX_E,
        matrix_nu=MATRIX_NU,
        matrix_friction_angle=MATRIX_FRICTION_ANGLE,
        matrix_flow_stress_ratio=MATRIX_FLOW_STRESS_RATIO,
        matrix_dilation_angle=MATRIX_DILATION_ANGLE,
        matrix_hardening_yield=MATRIX_HARDENING_YIELD,
        matrix_hardening_plastic_strain=MATRIX_HARDENING_PLASTIC_STRAIN,
        matrix_damage_strain=MATRIX_DAMAGE_STRAIN,
        matrix_damage_stress_triax=MATRIX_DAMAGE_STRESS_TRIAX,
        matrix_damage_strain_rate=MATRIX_DAMAGE_STRAIN_RATE,
        matrix_damage_displacement=MATRIX_DAMAGE_DISPLACEMENT,
        # 界面材料参数
        cohesive_K_nn=COHESIVE_K_NN,
        cohesive_K_ss=COHESIVE_K_SS,
        cohesive_K_tt=COHESIVE_K_TT,
        cohesive_t_n=COHESIVE_T_N,
        cohesive_t_s=COHESIVE_T_S,
        cohesive_t_t=COHESIVE_T_T,
        cohesive_GIC=COHESIVE_GIC,
        cohesive_GIIC=COHESIVE_GIIC,
        cohesive_GIIIC=COHESIVE_GIIIC,
        cohesive_eta=COHESIVE_ETA,
        cohesive_stab_coeff=COHESIVE_STAB_COEFF
    )

    # ========== 后处理 ==========
    if tempModelName in mdb.models:
        print("\n" + "=" * 70)
        print("Starting Model Cleanup and Rename...")
        print("=" * 70)

        models_to_delete = [m for m in mdb.models.keys()
                            if m != tempModelName]

        if models_to_delete:
            print("\nDeleting old models:")
            for m in models_to_delete:
                print("  - %s" % m)
            for modelKey in models_to_delete:
                del mdb.models[modelKey]
            print("Old models deleted.")
        else:
            print("\nNo old models to delete.")

        print("\nRenaming model:")
        print("  From: '%s'" % tempModelName)
        print("  To:   '%s'" % targetModelName)
        mdb.models.changeKey(fromName=tempModelName,
                             toName=targetModelName)

        print("\n" + "=" * 70)
        print("MODEL GENERATION COMPLETE")
        print("=" * 70)
    else:
        print("\n" + "!" * 70)
        print("ERROR: Model generation failed.")
        print("!" * 70 + "\n")