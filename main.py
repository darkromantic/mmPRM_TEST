import matplotlib.pyplot as plt
import numpy as np
import csv
import os

from utils.plotting import setup_plot, add_expand_obstacles, plot_start_and_end, add_real_obstacles, add_connect_obstacles, plot_nodes 
from utils.graph import generate_nodes, connect_nearby_nodes
from utils.path_planning import astar, smooth_path

SURFACE_POINTS = 500
PARAMETERS_CHOSEN = 4
WAY_POINTS = False

if PARAMETERS_CHOSEN == 1:
    from utils.parameters1 import get_parameters
elif PARAMETERS_CHOSEN == 2:
    from utils.parameters2 import get_parameters
elif PARAMETERS_CHOSEN == 3:
    from utils.parameters3 import get_parameters
elif PARAMETERS_CHOSEN == 4:
    from utils.parameters4 import get_parameters

def main():
    # setup the plot and obstacles
    params = get_parameters()
    ax, fig = setup_plot(params)
    expand_obstacles, obstacles_vertices = add_expand_obstacles(ax, params)
    real_obstacles, real_obstacles_vertices = add_real_obstacles(ax, params)
    connect_obstacles, real_obstacles_vertices = add_connect_obstacles(ax, params)
    plot_start_and_end(ax, params)

    # generate nodes
    samples, start_idx, end_idx, obstacles_surface_points = generate_nodes(
        params, connect_obstacles, total_points=SURFACE_POINTS, area_threshold=1, max_height=18, threshold_angle=45)
    

    # get waypoints and ensure they are part of the graph
    if WAY_POINTS :
        print('adding waypoints...')
        waypoints = params.get('waypoints', [])
        waypoint_nodes = [tuple(wp) for wp in waypoints]
        all_nodes_to_visit = [tuple(samples[start_idx])] + waypoint_nodes + [tuple(samples[end_idx])]
        existing_samples = {tuple(s) for s in samples}
        for node in waypoint_nodes:
            if node not in existing_samples:
                samples = np.vstack([samples, np.array(node)])
    else:
        waypoint_nodes = []
        all_nodes_to_visit = [tuple(samples[start_idx])] + [tuple(samples[end_idx])]

    # build the graph
    print("building graph...")
    edges, weights = connect_nearby_nodes(samples, params, expand_obstacles, obstacles_surface_points)
    print("graph built successfully!")

    # find the path
    final_path = []
    print("starting path planning...")
    for i in range(len(all_nodes_to_visit) - 1):
        start_node = all_nodes_to_visit[i]
        end_node = all_nodes_to_visit[i+1]
        print(f"  - planning path segment{i+1}: from {start_node} to {end_node}")
        path_segment = astar(start_node, end_node, edges, weights, params)
        if not path_segment:
            print(f"error: failed to find path segment from {start_node} to {end_node}.")
            plt.show()
            return 
        if not final_path:
            final_path.extend(path_segment)
        else:
            final_path.extend(path_segment[1:])

    path = final_path
    if not path:
        print("path not found!")
        plt.show()
        return
    
    print("path found successfully!")

    # plot the path
    print("plotting path...")
    data_rows = []
    wg_path = []
    wf_path = []
    for i in range(len(path) - 1):
        wg = params['wg']
        wf = params['wf']
        start_point = path[i]
        end_point = path[i + 1]
        edge = frozenset([start_point, end_point])
        weight = weights[edge]
        if end_point != path[-1]:
            edge_1 = frozenset([path[i + 1], path[i + 2]])
        else:
            edge_1 = None

        if weight == wg:
            wg_path.append(start_point)
            if not edge_1 or weights[edge_1] != wg or end_point in waypoint_nodes:
                wg_path.append(end_point)
                wg_smoothed_path = smooth_path(wg_path, type="wg")
                for i in range(len(wg_smoothed_path) - 1):
                    start_point = wg_smoothed_path[i]
                    end_point = wg_smoothed_path[i + 1]
                    ax.plot(
                        [start_point[0], end_point[0]],
                        [start_point[1], end_point[1]],
                        [start_point[2], end_point[2]],
                        color='blue',
                        linestyle='-',
                        linewidth=2.0,
                    )
                for point in wg_smoothed_path:
                    data_rows.append([str(point), "wg"])
                wg_path = []

        else:
            wf_path.append(start_point)
            if not edge_1 or weights[edge_1] == wg or end_point in waypoint_nodes:
                wf_path.append(end_point)
                wf_smoothed_path = smooth_path(wf_path, type='wf')
                for i in range(len(wf_smoothed_path) - 1):
                    start_point = wf_smoothed_path[i]
                    end_point = wf_smoothed_path[i + 1]
                    ax.plot(
                        [start_point[0], end_point[0]],
                        [start_point[1], end_point[1]],
                        [start_point[2], end_point[2]],
                        color='red',
                        linestyle='-',
                        linewidth=2.0,
                    )
                for point in wf_smoothed_path:
                    data_rows.append([point, "wf"])
                wf_path = []
    
    ax.legend()
    plt.show()
    return data_rows, params

def save_to_csv(data_rows, params):
    print("saving path to CSV...")
    if PARAMETERS_CHOSEN == 1:
        output_folder = 'test1_csv'
        print(f"output folder: {output_folder}")
    elif PARAMETERS_CHOSEN == 2:
        output_folder = 'test2_csv'
    else:
        output_folder = f'test{PARAMETERS_CHOSEN}_csv'

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    csv_name = f"omega_t{params['omega_t']}_omega_e{params['omega_e']}_points{SURFACE_POINTS}.csv"
    full_path = os.path.join(output_folder, csv_name)
    with open(full_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['points', 'labels'])  
        writer.writerows(data_rows)
    print(f"CSV saved to {full_path}")

def node_graduation_test(IS_PLOT_EDGES=False):
    # setup the plot and obstacles
    params = get_parameters()
    ax, fig = setup_plot(params)
    expand_obstacles, obstacles_vertices = add_expand_obstacles(ax, params)
    real_obstacles, real_obstacles_vertices = add_real_obstacles(ax, params)
    connect_obstacles, real_obstacles_vertices = add_connect_obstacles(ax, params)
    plot_start_and_end(ax, params)

    # generate nodes
    samples, start_idx, end_idx, obstacles_surface_points = generate_nodes(
        params, connect_obstacles, total_points=SURFACE_POINTS, area_threshold=1, max_height=18, threshold_angle=45)
    plot_nodes(ax, samples, obstacles_surface_points, IS_PLOT_EDGES)

    if IS_PLOT_EDGES:
        # build the graph
        print("building graph...")
        edges, weights = connect_nearby_nodes(samples, params, expand_obstacles, obstacles_surface_points)
        print("graph built successfully!")
        # plot the edges
        for edge, weight in weights.items():  
            edge = list(edge)
            start_point = edge[0]
            end_point = edge[1]
            ax.plot(
                [start_point[0], end_point[0]],
                [start_point[1], end_point[1]],
                [start_point[2], end_point[2]],
                color='black',
                linestyle='-',
                linewidth=.5,
            )
    plt.show()



if __name__ == "__main__":
    # node_graduation_test(IS_PLOT_EDGES=False)
    data_rows, params = main()
    if data_rows:
        save_to_csv(data_rows, params)