import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import matplotlib.ticker as ticker
import matplotlib.font_manager as fm # 导入字体管理器

def setup_plot(params, tick_interval=5):
  
    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection='3d')

    # # --- 设置图表标题 ---
    # ax.set_title("3D Path Visualization", fontsize=10) # 标题 Arial 10号 (如果Arial可用)

    ax.set_xlim(params['xlim'])
    ax.set_ylim(params['ylim'])
    ax.set_zlim(params['zlim'])

    # # --- 坐标轴标签字体 (Arial 8号) ---
    # ax.set_xlabel('X Axis', fontsize=8)
    # ax.set_ylabel('Y Axis', fontsize=8)
    # ax.set_zlabel('Z Axis', fontsize=8)

    # # --- 刻度标签字体 (Arial 8号) ---
    # ax.tick_params(axis='x', labelsize=8)
    # ax.tick_params(axis='y', labelsize=8)
    # ax.tick_params(axis='z', labelsize=8)

    ax.set_xticks([params['xlim'][0] + i * tick_interval for i in range(int((params['xlim'][1] - params['xlim'][0]) // tick_interval) + 1)])
    ax.set_yticks([params['ylim'][0] + i * tick_interval for i in range(int((params['ylim'][1] - params['ylim'][0]) // tick_interval) + 1)])
    ax.set_zticks([params['zlim'][0] + i * tick_interval for i in range(int((params['zlim'][1] - params['zlim'][0]) // tick_interval) + 1)])

    ax.xaxis.set_major_formatter(ticker.NullFormatter())
    ax.yaxis.set_major_formatter(ticker.NullFormatter())
    ax.zaxis.set_major_formatter(ticker.NullFormatter())

    ax.set_box_aspect([abs(lim[1] - lim[0]) for lim in [params['xlim'], params['ylim'], params['zlim']]])
    return ax, fig


def add_expand_obstacles(ax, params):
    obstacles = []
    obstacles_vertices = []
    for obstacle_params in params['expand_obstacles']:
        faces, vertices = create_polyhedron(obstacle_params['vertices'], obstacle_params['faces'])
        obstacles.append(faces)
        obstacles_vertices.append(vertices)
        # ax.add_collection3d(Poly3DCollection(faces, facecolors='red', linewidths=1.0, edgecolors=[0.8, 0.2, 0.2], alpha=.1, linestyle='--'))
    return obstacles, obstacles_vertices

def add_connect_obstacles(ax, params):
    real_obstacles = []
    real_obstacles_vertices = []
    for obstacle_params in params['connect_obstacles']:
        faces, vertices = create_polyhedron(obstacle_params['vertices'], obstacle_params['faces'])
        real_obstacles.append(faces)
        real_obstacles_vertices.append(vertices)
        # ax.add_collection3d(Poly3DCollection(faces, facecolors='blue', linewidths=1.5, edgecolors=[0.2, 0.2, 0.8], alpha=.1, linestyle='-.'))
    return real_obstacles, real_obstacles_vertices

def add_real_obstacles(ax, params):
    real_obstacles = []
    real_obstacles_vertices = []
    for obstacle_params in params['real_obstacles']:
        faces, vertices = create_polyhedron(obstacle_params['vertices'], obstacle_params['faces'])
        real_obstacles.append(faces)
        real_obstacles_vertices.append(vertices)
        ax.add_collection3d(Poly3DCollection(
            faces,
            facecolors='grey',
            linewidths=1.5,
            edgecolors=[0.1, 0.1, 0.1],
            alpha=1.0,
            linestyle='-',
            zorder=0
        ))
    return real_obstacles, real_obstacles_vertices

def plot_start_and_end(ax, params, NO_SECOND_WAYPOINT=False,IS_LAST_WAYPOINT=False):
    # 绘制起点和终点
    ax.scatter(*params['start'], color='green', s=200, label='Start')
    ax.scatter(*params['end'], color='blue', s=200, label='End', marker="*")

    # 绘制路径点
    waypoints = params.get('waypoints', [])
    if waypoints:
        for i, wp in enumerate(waypoints):
            # 跳过第二个路径点 (索引为 1)
            if  NO_SECOND_WAYPOINT and i == 1:
                continue
            if not IS_LAST_WAYPOINT and i == len(waypoints) - 1:
                continue
            # 使用红色的三角形标记代表小红旗
            ax.scatter(wp[0], wp[1], wp[2],
                       color='red', s=80, label='Waypoint' if i == 0 else "_nolegend_", # 只为第一个旗子添加图例标签
                       marker='^') 
            # # 可选：为旗子添加旗杆 (如果需要更精细的旗帜外观)
            # pole_height = 2 # 旗杆相对于标记点的高度（或深度，取决于你的坐标系）
            # ax.plot([wp[0], wp[0]], [wp[1], wp[1]], [wp[2], wp[2] - pole_height], color='black', linewidth=1)


    # --- 图例字体 (Arial 8号) ---
    # 确保图例只在有可显示标签时才创建
    # handles, labels = ax.get_legend_handles_labels()
    # if handles: 
    #     ax.legend(fontsize=8)

def create_polyhedron(vertices, faces_index):
    vertices = np.array(vertices)
    faces = []
    for face_index in faces_index:
        face_points = vertices[face_index]
        faces.append(face_points)

    faces = [np.array(face) for face in faces]
    return faces, vertices.tolist()
