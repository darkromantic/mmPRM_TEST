from utils.plotting import setup_plot, add_real_obstacles, plot_start_and_end, add_connect_obstacles, add_expand_obstacles
import matplotlib.pyplot as plt
import glob
import os
import pandas as pd
import numpy as np
from scipy.interpolate import splprep, splev

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
ax, fig = setup_plot(params)
waypoints = params.get('waypoints', [])
waypoint_nodes = [tuple(wp) for wp in waypoints]

# 添加障碍物和起点终点
expand_obstacles, obstacles_vertices = add_expand_obstacles(ax, params)
real_obstacles, real_obstacles_vertices = add_real_obstacles(ax, params)
connect_obstacles, real_obstacles_vertices = add_connect_obstacles(ax, params)
plot_start_and_end(ax, params, NO_SECOND_WAYPOINT, PLOT_WAYPOINT=PLOT_WAYPOINTS)

COLOR_MAP = {"wg": "blue", "wf": "red"}

def parse_point(point_str):
    stripped_str = point_str.strip().replace("(", "").replace(")", "").replace("[", "").replace("]", "")
    coords = [part.strip() for part in stripped_str.split() if part.strip()]
    coords = [c + "0" if c.endswith(".") else c for c in coords]
    return list(map(float, coords))

def interpolate_wf_segment(segment):
    """对wf标签的路径段进行B样条插值"""
    path_array = np.array(segment)
    x, y, z = path_array[:, 0], path_array[:, 1], path_array[:, 2]
    
    # 样条参数配置
    length = len(x)
    spline_degree = min(3, length-1)  # 最大3次样条，根据点数自动降阶
    smoothing_factor = 10  # 控制平滑度
    # 添加额外的控制点以确保起点和终点处的切线沿z轴方向
    start_point = segment[0]
    end_point = segment[-1]
    
    # 定义一个小增量，用于创建沿z轴方向的切线效果
    epsilon = 0.5  # 增加到0.5以增强效果
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

    tck, u = splprep([x, y, z], s=smoothing_factor, k=spline_degree)
    u_new = np.linspace(u.min(), u.max(), num=100)  # 生成100个插值点
    return splev(u_new, tck)


def plot_segment(df, linewidth=2.0, alpha=0.7):
    """绘制路径段"""
    current_label = None
    current_segment = []
    coords_before = None
    for _, row in df.iterrows():
        coords = row['coordinates']
        label = row['labels']
        if coords_before == coords:
            continue
        print(f" point before: {coords_before}, point now: {coords}, label: {label}, current_label: {current_label}")
        if label != current_label or tuple(coords) in waypoint_nodes:
            if current_segment:
                current_segment.append(coords)  # 包含路径点
                print(">>>>draw segment>>>>")
                print(f"segment_start: {current_segment[0]}, segment_end: {current_segment[-1]}, segment_label: {current_label}\n")
                # 绘制前一个段
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
            # 如果是最后一个点，直接添加到当前段
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

if PARAMETERS_CHOSEN == 1:
    folder_path = 'test1_csv'
elif PARAMETERS_CHOSEN == 2: 
    folder_path = 'test2_csv'
else:
    folder_path = f'test{PARAMETERS_CHOSEN}_csv'

all_files = glob.glob(os.path.join(folder_path, "*.csv"))
for filepath in all_files:
    filename = os.path.basename(filepath)
    df = pd.read_csv(filepath) # 读取CSV文件
    df['coordinates'] = df['points'].apply(parse_point)
    if filename.startswith('H'):
        plot_segment(df, linewidth=2.0, alpha=1)
    elif TEST_ALL_FILES:
        plot_segment(df, linewidth=1.0, alpha=0.3)


# 创建图例
# legend_elements = [
#     plt.Line2D([0], [0], color=COLOR_MAP['wg'], label='Global Path'),
#     plt.Line2D([0], [0], color=COLOR_MAP['wf'], label='Optimized Path')
# ]
# ax.legend(handles=legend_elements)
ax.view_init(elev=8, azim=-70,roll=0)
ax.text2D(0.05, 0.7, f"omega_e: {params['omega_e']}, omega_t: {params['omega_t']}", transform=ax.transAxes, fontsize=10)
plt.show()