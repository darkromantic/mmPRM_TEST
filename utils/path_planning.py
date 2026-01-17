import numpy as np
from scipy.interpolate import splprep, splev

# 计算节点间距离
def distance(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

def kappa_cost(omega, difference_total, c_cost):
    return np.exp(( omega/difference_total) * c_cost)

def distance_xy(point1, point2) -> float:
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

def distance_z(point1, point2) -> float:
    return abs(point1[2] - point2[2])

# MM_A* 算法
def astar(start, goal, edges, weights, params):
    wg = params['wg']
    wf= params['wf']
    pc = params.get('pc')
    pf = params.get('pf')
    m = params['m']
    g = params.get('g', 9.81)
    vc = params.get('vc')
    vf = params.get('vf')
    et = params.get('et')
    Wt = params.get('Wt')
    omega_e = params.get('omega_e')
    omega_t = params.get('omega_t')

    def heuristic_t(point1, point2):
        if point1 == point2:
            return 0
        edge = frozenset([point1, point2])
        weight = weights[edge]
        if isinstance(weight, list):
            vm = (vc + vf) / 2
        if weight == wg:
            vm = vc
        else:
            vm = vf
        return distance(point1, point2) / vm
    
    def heuristic_e(point1, point2):
        if point1 == point2:
            return 0
        edge = frozenset([point1, point2])
        weight = weights[edge]
        if weight == wg:
            return distance_xy(point1, point2) * pc/vc
        else:
            return distance_xy(point1, point2) * pf/vf + distance_z(point1, point2) * m * g 
        
    def heuristic_H_e(point, goal):
        return pc * distance_xy(point, goal)/vc + pf * distance_z(point, goal)/vf + m * g * distance_z(point, goal) + Wt
    
    def heuristic_H_t(point, goal):
        return  distance_xy(point, goal)/vc + distance_z(point, goal)/vf + et

    def g_fun(point1, point2):
        return kappa_cost(omega_e, heuristic_H_e(start, goal), heuristic_e(point1, point2)) + kappa_cost(omega_t, heuristic_H_t(start, goal), heuristic_t(point1, point2))
    
    def h_fun(point, goal):
        return kappa_cost(omega_e, heuristic_H_e(start, goal), heuristic_H_e(point, goal)) + kappa_cost(omega_t, heuristic_H_t(start, goal), heuristic_H_t(point, goal))
    
    open_set = set()
    closed_set = set()
    came_from = {}
    g_score = {start: 0}
    f_score = {start: h_fun(start, goal)}

    open_set.add(start)

    while open_set:
        current = min(open_set, key=lambda o: f_score[o])

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        open_set.remove(current)
        closed_set.add(current)

        for neighbor in edges[current]:
            tentative_g_score = g_score[current] + g_fun(current, neighbor)

            if neighbor in closed_set:
                continue

            if neighbor not in open_set:
                open_set.add(neighbor)

            if tentative_g_score >= g_score.get(neighbor, float('inf')):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g_score
            f_score[neighbor] = tentative_g_score + h_fun(neighbor, goal)

    return None

def smooth_path(path, type, smoothing_factor=5, spline_degree=3):
    """使用B样条插值平滑给定路径，并确保起点和终点处的切线沿z轴方向且更加明显。
    
    参数:
        path (list of tuples): 输入路径点，每个点为(x, y, z)格式。
        type (str): 类型标识符，"wf" 表示添加沿z轴方向的切线条件。
        smoothing_factor (float): 平滑因子，值越小曲线越接近原始数据点。
        spline_degree (int): 样条次数，默认为3（三阶）。
    
    返回:
        ndarray: 平滑后的路径点。
    """
    
    path_array = np.array(path)
    x, y, z = path_array[:, 0], path_array[:, 1], path_array[:, 2]

    # 更新路径长度
    length = len(x)
    
    if type == "wf":
        # 添加额外的控制点以确保起点和终点处的切线沿z轴方向
        start_point = path[0]
        end_point = path[-1]
        
        # 定义一个小增量，用于创建沿z轴方向的切线效果
        epsilon = 0.25  # 增加到0.5以增强效果
        num_control = int(len(z)/3)

        for j in range(len(z)-2):
            z[j+1] = z[j+1] + num_control * epsilon
           
            

        for i in range(num_control): 
            # 在起点附近添加多个控制点，使切线沿正z轴方向
            x = np.insert(x, i+1, start_point[0])
            y = np.insert(y, i+1, start_point[1])
            z = np.insert(z, i+1, start_point[2] + (i+1)*epsilon)
        
            # 在终点附近添加多个控制点，使切线沿负z轴方向（假设你想要相反方向）
            x = np.insert(x, -(i+1), end_point[0])
            y = np.insert(y, -(i+1), end_point[1])
            z = np.insert(z, -(i+1), end_point[2] + (i+1) * epsilon)
        # 给起始点和终点附近的控制点赋予更高的权重
        weights = np.ones(len(x))
        weights[:num_control] = 5  # 起始点附近权重设为5
        weights[-(num_control+1):] = 5  # 结束点附近权重设为5
        smoothed_path = np.column_stack((x, y, z))
        return smoothed_path
        
    else:
        weights = None

    
    if length <= spline_degree:
        spline_degree = max(1, length - 1)  # 确保样条次数至少为1
    
    tck, u = splprep([x, y, z], s=smoothing_factor, per=False, k=spline_degree, w=weights)
    
    # 生成新的参数值以获得更高分辨率的路径
    u_new = np.linspace(u.min(), u.max(), num=length * 10)
    x_new, y_new, z_new = splev(u_new, tck, der=0)
    
    smoothed_path = np.column_stack((x_new, y_new, z_new))
    return smoothed_path