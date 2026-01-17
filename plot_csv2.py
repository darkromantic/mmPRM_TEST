from utils.plotting import setup_plot, add_real_obstacles, plot_start_and_end, add_connect_obstacles, add_expand_obstacles
import matplotlib.pyplot as plt
import glob
import os
import pandas as pd
import numpy as np
from scipy.interpolate import splprep, splev
import re
import ast

PLOT_WAYPOINTS = False
NO_SECOND_WAYPOINT = False    
PARAMETERS_CHOSEN = 4
TEST_ALL_FILES = False  

if PARAMETERS_CHOSEN == 1:
    from utils.parameters1 import get_parameters
elif PARAMETERS_CHOSEN == 2:
    from utils.parameters2 import get_parameters
elif PARAMETERS_CHOSEN == 3:
    from utils.parameters3 import get_parameters
elif PARAMETERS_CHOSEN == 4:
    from utils.parameters4 import get_parameters

params = get_parameters()
COLOR_MAP = {"wg": "blue", "wf": "red"}
waypoints = params.get('waypoints', [])
waypoint_nodes = [tuple(wp) for wp in waypoints]

def parse_point(point_str):
    stripped_str = point_str.strip().replace("(", "").replace(")", "").replace("[", "").replace("]", "")
    coords = [part.strip() for part in stripped_str.split() if part.strip()]
    coords = [c + "0" if c.endswith(".") else c for c in coords]
    return list(map(float, coords))

def interpolate_wf_segment(segment):
    """对wf标签的路径段进行B样条插值"""
    path_array = np.array(segment)
    x, y, z = path_array[:, 0], path_array[:, 1], path_array[:, 2]
    
    length = len(x)
    spline_degree = min(3, length-1)
    smoothing_factor = 10
    
    if length > 3:
        tck, u = splprep([x, y, z], s=smoothing_factor, k=spline_degree)
        u_new = np.linspace(u.min(), u.max(), num=100)
        return splev(u_new, tck)
    else:
        return x, y, z

def plot_segment(df, ax, linewidth=2.0, alpha=0.7):
    """绘制路径段"""
    current_label = None
    current_segment = []
    coords_before = None
    
    df['coordinates'] = df['points'].apply(parse_point)
    
    for _, row in df.iterrows():
        coords = row['coordinates']
        label = row['labels']
        if coords_before == coords:
            continue
        
        print(f" point before: {coords_before}, point now: {coords}, label: {label}, current_label: {current_label}")
        
        if label != current_label or coords in waypoint_nodes:
            if current_segment:
                current_segment.append(coords)
                print(">>>>draw segment>>>>")
                print(f"segment_start: {current_segment[0]}, segment_end: {current_segment[-1]}, segment_label: {current_label}\n")
                
                if current_label == "wf":
                    xs, ys, zs = interpolate_wf_segment(current_segment)
                else:
                    xs, ys, zs = zip(*current_segment)

                ax.plot(xs, ys, zs, 
                       color=COLOR_MAP[current_label],
                       linestyle='-', 
                       linewidth=linewidth,
                       alpha=alpha,
                       zorder=10)
                current_segment = []
            
            current_label = label
            current_segment = [coords]
        
        elif coords == df['coordinates'].iloc[-1]:
            current_segment.append(coords)
            print(">>>>draw segment>>>>")
            print(f"segment_start: {current_segment[0]}, segment_end: {current_segment[-1]}, segment_label: {current_label}\n")
            if current_label == "wf":
                xs, ys, zs = interpolate_wf_segment(current_segment)
            else:
                xs, ys, zs = zip(*current_segment)

            ax.plot(xs, ys, zs, 
                   color=COLOR_MAP[current_label],
                   linestyle='-', 
                   linewidth=linewidth,
                   alpha=alpha,
                   zorder=10)
        else:
            current_segment.append(coords)
        coords_before = coords

def take_omega(filename):
    """
    从文件名中提取 omega_t, omega_e 和 points 参数。
    """
    pattern = r"omega_t([\d.]+)_omega_e([\d.]+)_points(\d+)\.csv"
    match = re.search(pattern, filename)

    if match:
        omega_t = float(match.group(1))
        omega_e = float(match.group(2))
        points = int(match.group(3))
        return omega_t, omega_e, points
    else:
        print(f"文件名格式不匹配，无法提取参数: {filename}")
        return None, None, None

# --- 主要执行部分 ---
if PARAMETERS_CHOSEN == 1:
    folder_path = 'test1_csv'
elif PARAMETERS_CHOSEN == 2: 
    folder_path = 'test2_csv'
else:
    folder_path = f'test{PARAMETERS_CHOSEN}_csv'

all_files = glob.glob(os.path.join(folder_path, "*.csv"))

# **关键修改**：在循环外只创建一次图表

for filepath in all_files:
    # **关键修改**：在每次循环开始时清空图表内容
    ax, fig = setup_plot(params)
    
    # 重新添加障碍物和起点终点
    expand_obstacles, obstacles_vertices = add_expand_obstacles(ax, params)
    real_obstacles, real_obstacles_vertices = add_real_obstacles(ax, params)
    connect_obstacles, real_obstacles_vertices = add_connect_obstacles(ax, params)
    plot_start_and_end(ax, params, NO_SECOND_WAYPOINT, PLOT_WAYPOINT=PLOT_WAYPOINTS)

    df = pd.read_csv(filepath)
    plot_segment(df, ax, linewidth=2.0, alpha=1)
    
    ax.view_init(elev=8, azim=-70,roll=0)
    
    filename = os.path.basename(filepath)
    omega_t, omega_e, points = take_omega(filename)
    
    if omega_t is not None:
        ax.set_title(f"omega_t: {omega_t}, omega_e: {omega_e}")
        # ax.text2D(...) # set_title 更适合作为标题
    
    # **关键修改**：保留 plt.show() 在循环内部
    plt.show()


# High light compare
def high_light_compare(filepath):
    filename = os.path.basename(filepath)
    df = pd.read_csv(filepath) 
    df['coordinates'] = df['points'].apply(parse_point)
    if filename.startswith('H'):
        plot_segment(df, ax, linewidth=2.0, alpha=1)
    elif TEST_ALL_FILES:
        plot_segment(df, ax, linewidth=1.0, alpha=0.3)