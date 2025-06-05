import numpy as np
import random
from scipy.spatial import ConvexHull
from collections import defaultdict

# 生成节点
def generate_nodes(params, obstacles_faces, total_points, area_threshold, max_height, threshold_angle):
    num_samples = params['num_samples']
    num_node_g = int(params['ground_ratio'] * num_samples)
    num_node_f = num_samples - num_node_g

    # 生成地面节点（z=0）
    node_g = np.random.uniform(
        low=[params['xlim'][0], params['ylim'][0]],
        high=[params['xlim'][1], params['ylim'][1]],
        size=(num_node_g, 2)
    )
    node_g = np.hstack((node_g, np.zeros((num_node_g, 1))))  # 地面节点 z=0
    node_g = np.vstack([node_g, params['start']])  # 添加起点
    start_idx = len(node_g) - 1

    # 生成障碍物表面节点
    obstacles_surface_points = generate_points_on_prism(obstacles_faces,  total_points, area_threshold, max_height, threshold_angle)


    # 生成空中节点（z>0）
    node_f = np.random.uniform(
        low=[params['xlim'][0], params['ylim'][0], params['zlim'][0]],
        high=[params['xlim'][1], params['ylim'][1], params['zlim'][1]],
        size=(num_node_f, 3)
    )
    node_f = np.vstack([node_f, params['end']])  # 添加终点
    end_idx = len(node_g) + len(node_f) - 1

    # 合并地面节点、空中节点和障碍物表面节点
    samples = np.vstack((node_g, node_f, obstacles_surface_points))
    obstacles_surface_points = np.vstack([obstacles_surface_points, params['end']])  # 添加终点
    return samples, start_idx, end_idx, obstacles_surface_points

def generate_points_on_prism(obstacles_faces, total_points, 
                           area_threshold, height_threshold, threshold_angle):
    
    # ---------------------- 内部辅助函数 ----------------------
    
    def calculate_normal(points):
        """计算平面法向量"""
        v1 = points[1] - points[0]
        v2 = points[2] - points[0]
        normal = np.cross(v1, v2)
        return normal / np.linalg.norm(normal)
    
    def is_valid_face(face_points):
        """判断是否是需要生成点的有效面"""
        # 排除底面（z=0的面）
        face_points = np.array(face_points)
        if np.allclose(face_points[:,2], 0):
            return False
        
        # 检查高度限制
        if np.max(face_points[:,2]) > height_threshold:
            return False
        
        # 检查平面倾斜角度
        normal = calculate_normal(face_points)
        vertical = np.array([0,0,1])
        angle = np.degrees(np.arccos(np.clip(np.dot(normal, vertical), -1, 1)))
        return angle < threshold_angle
    
    def polygon_area(points):
        """计算多边形面积（三维投影到XY平面）"""
        x = points[:,0]
        y = points[:,1]
        return 0.5 * np.abs(np.dot(x, np.roll(y,1)) - np.dot(y, np.roll(x,1)))
    
    def triangulate_quad(points):
        """将四边形分解为两个三角形（优化对角线选择）"""
        # 选择使三角形更接近等积的对角线
        area1 = polygon_area(points[[0,1,2]]) + polygon_area(points[[0,2,3]])
        area2 = polygon_area(points[[0,1,3]]) + polygon_area(points[[1,2,3]])
        return [[0,1,2], [0,2,3]] if area1 <= area2 else [[0,1,3], [1,2,3]]
    
    def generate_triangle_points(A, B, C, n):
        """在三角形ABC内生成均匀分布的点"""
        points = []
        for _ in range(n):
            # 改进的均匀采样方法
            r1, r2 = np.random.rand(2)
            if r1 + r2 > 1:
                r1 = 1 - r1
                r2 = 1 - r2
            points.append(A + r1*(B-A) + r2*(C-A))
        return np.array(points)
    
    # ---------------------- 主逻辑 ----------------------

    # 筛选有效面
    valid_faces = []
    face_areas = []
    for faces_points in obstacles_faces:
        for face_points in faces_points:
            if is_valid_face(face_points):
                print('通过角度测试')
                area = polygon_area(face_points)
                if area > area_threshold:
                    print('通过面积测试')
                    valid_faces.append(face_points)
                    face_areas.append(area)
    
    # 按面积比例分配点数
    total_area = sum(face_areas)
    points = []
    for face, area in zip(valid_faces, face_areas):
        # 将四边形分解为三角形
        triangles = triangulate_quad(face) if len(face) == 4 else [range(len(face))]
        tri_areas = [polygon_area(face[t]) for t in triangles]
        tri_points = [int(total_points * a / total_area) for a in tri_areas]
        
        # 在每个三角形中生成点
        for t, n in zip(triangles, tri_points):
            A, B, C = face[t[0]], face[t[1]], face[t[2]]
            points.extend(generate_triangle_points(A, B, C, n))
    
    return np.array(points)[:total_points]  # 确保总数准确





# 连接邻近节点
def connect_nearby_nodes(samples, params, obstacles, obstacle_surface_points):

    weights = {}
    wg = params['wg']
    wf = params['wf']
    e = 10 * params['e_factor']
    edges = defaultdict(set)

    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            distance = heuristic(samples[i], samples[j])
            edge_key = frozenset([tuple(samples[i]), tuple(samples[j])])

            if distance <= params['R_max'] and not check_collision(samples[i], samples[j], obstacles):
                edges[tuple(samples[i])].add(tuple(samples[j]))
                edges[tuple(samples[j])].add(tuple(samples[i]))

                if samples[i][2] == 0 and samples[j][2] == 0:
                    weights[edge_key] = wg
                elif samples[i][2] > 0 and samples[j][2] > 0:
                    if samples[i] in obstacle_surface_points:
                        if samples[j] in obstacle_surface_points:
                            weights[edge_key] = wg
                            print('平面节点生成成功')
                        else:
                            weights[edge_key] = [wf, e]
                    elif samples[i] not in obstacle_surface_points:
                        if samples[j] in obstacle_surface_points:
                            weights[edge_key] = [wf, e]
                        else:
                            weights[edge_key] = wf
                else:
                    weights[edge_key] = [wf, e]

    return edges, weights

# 计算节点间距离
def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


# 判断点是否在凸多面体的内部
def is_inside_convex_polyhedron(point, obstacle):
    # 提取所有唯一的点
    points = np.unique(np.array([point for face in obstacle for point in face]), axis=0)
    
    # 计算这些点的凸包
    hull = ConvexHull(points)
    
    # 检查点是否在凸包内（不包括面上）
    def is_point_in_hull(point, hull):
        return all(np.dot(eq[:-1], point) + eq[-1] < 0 for eq in hull.equations)
    
    return is_point_in_hull(point, hull)

def check_collision(line_start, line_end, obstacles, angle_threshold=50):
    EPSILON = 1e-6


    # 射线-平面相交检测
    def ray_plane_intersection(ro, rd, plane):
        """
        射线-平面相交检测。如果线段位于平面上，仍视为相交。
    
        :param ro: 射线起点 (NumPy 数组，3D 坐标)
        :param rd: 射线方向 (NumPy 数组，3D 向量)
        :param plane: 平面顶点列表 (列表，包含 3 个顶点)
        :return: 交点坐标 (NumPy 数组，3D 坐标) 或 None（无交点）
        """
        EPSILON = 1e-6  # 容差值
        plane = np.array(plane)  # 将平面顶点转换为 NumPy 数组
    
        # 计算面法向量
        v1 = plane[1] - plane[0]
        v2 = plane[2] - plane[0]
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)
    
        # 检查线段的两个端点是否都在平面上
        start_on_plane = np.abs(np.dot(ro - plane[0], normal)) < EPSILON
        end_on_plane = np.abs(np.dot((ro + rd) - plane[0], normal)) < EPSILON
        if start_on_plane and end_on_plane:
            return ro  # 线段位于平面上，返回起点
    
        # 计算线段与面的交点
        denom = np.dot(normal, rd)
        if abs(denom) < EPSILON:
            if start_on_plane :
                return ro
            elif end_on_plane:
                rn = ro + rd
                return rn
            return None  # 平行于平面且不在平面上
    
        t = np.dot(plane[0] - ro, normal) / denom
        if t < 0 or t > 1:
            return None  # 交点不在线段内
    
        return ro + rd * t


    def is_on_face_projection(point, face):
        EPSILON = 1e-6
      
        # 计算面法向量
        if len(face) < 3:
            return False  # 无效面
          
        v0 = face[0]
        v1 = face[1] - v0
        v2 = face[2] - v0
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
      
        if norm < EPSILON:
            return False  # 退化面
          
        normal = normal / norm
      
        # 选择投影轴（避免投影到与法向量垂直的平面）
        axis = np.argmax(np.abs(normal))  # 选择法向量最大的分量轴
        u = (axis + 1) % 3  # 确定二维投影坐标
        v = (axis + 2) % 3
      
        # 生成投影坐标系
        proj_matrix = np.array([
            [u==0, u==1, u==2],  # u轴选择器
            [v==0, v==1, v==2]   # v轴选择器
        ], dtype=float)
      
        # 投影所有顶点到2D平面
        poly2d = [proj_matrix @ vertex for vertex in face]
        px, py = proj_matrix @ point

        wn = 0
        for i in range(len(poly2d)):
            p1 = poly2d[i]
            p2 = poly2d[(i+1) % len(poly2d)]
            # 跳过水平边
            if abs(p2[1] - p1[1]) < EPSILON:
                continue
            # 计算交点参数
            t = (py - p1[1]) / (p2[1] - p1[1])  # 移除EPSILON
            intersect_x = p1[0] + t * (p2[0] - p1[0])
            # 绕数计算
            if p1[1] <= py:
                # 检查向上的边，交点是否在右侧
                if p2[1] > py and intersect_x > px:
                    wn += 1
            else:
                # 检查向下的边，交点是否在右侧
                if p2[1] <= py and intersect_x > px:
                    wn -= 1
        return wn != 0
    

    # 计算平面法向量与参考向量的夹角
    def calculate_angle(normal):
        # 参考向量（世界坐标系的Z轴）
        reference_vector = np.array([0, 0, 1])
        # 计算夹角
        cos_theta = np.dot(normal, reference_vector) / (np.linalg.norm(normal) * np.linalg.norm(reference_vector))
        angle = np.arccos(cos_theta) * 180 / np.pi  # 转换为角度
        return angle

    # 主检测逻辑
    line_start = np.array(line_start)
    line_end = np.array(line_end)
    direction = line_end - line_start

    # 检查线段是否完全位于多面体的内部
    for obstacle in obstacles:
        # 检查起点和终点是否都在多面体的内部
        if is_inside_convex_polyhedron(line_start, obstacle) or is_inside_convex_polyhedron(line_end, obstacle):
            return True

    # 遍历所有障碍物和面
    for obstacle in obstacles:
        for face in obstacle:
            face = np.array(face)
            # 计算面法向量
            v1 = face[1] - face[0]
            v2 = face[2] - face[0]
            normal = np.cross(v1, v2)
            normal = normal / np.linalg.norm(normal)
          
            # 计算与参考向量的夹角
            angle = calculate_angle(normal)
            
          
            # 检测线段与面的相交
            intersection = ray_plane_intersection(line_start, direction, face)

            if intersection is not None:
                 # 如果相交，判断交点是否在面的投影内
                if is_on_face_projection(intersection, face):
            
                    if np.array_equal(intersection, line_start) or np.array_equal(intersection, line_end):     
                        
                        # 如果平面法向量夹角小于等于阈值，则不视为碰撞
                        if angle <= angle_threshold:
                           
                            # 如果在地面上，则视为碰撞
                            if intersection[2] ==0:
                                
                                return True
                            continue
                        else:
                            return True     
                    else:
                        return True 

               

    return False