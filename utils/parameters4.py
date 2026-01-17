def create_cuboid_obstacle(min_corner, max_corner):
    """
    根据长方体的最小和最大角点坐标生成障碍物字典。
    
    Args:
        min_corner (tuple): 长方体的最小坐标 (x_min, y_min, z_min)。
        max_corner (tuple): 长方体的最大坐标 (x_max, y_max, z_max)。
        
    Returns:
        dict: 包含 'vertices' 和 'faces' 的字典。
    """
    x_min, y_min, z_min = min_corner
    x_max, y_max, z_max = max_corner
    
    # 定义8个顶点
    vertices = [
        [x_min, y_max, z_max],  # 0
        [x_min, y_min, z_max],  # 1
        [x_max, y_min, z_max],  # 2
        [x_max, y_max, z_max],  # 3
        [x_min, y_max, z_min],  # 4 (底面)
        [x_min, y_min, z_min],  # 5 (底面)
        [x_max, y_min, z_min],  # 6 (底面)
        [x_max, y_max, z_min]   # 7 (底面)
    ]
    
    # 定义6个面，面的顶点索引顺序决定了法线方向
    # 这里的顺序与您原代码中的顺序略有不同，但定义的是同一个立方体
    faces = [
        [0, 1, 2, 3],  # 顶面
        [4, 5, 6, 7],  # 底面
        [0, 1, 5, 4],  # 左面
        [2, 3, 7, 6],  # 右面
        [0, 3, 7, 4],  # 前面
        [1, 2, 6, 5]   # 后面
    ]
    
    return {'vertices': vertices, 'faces': faces}

def create_sloped_obstacle(x_range, y_range, z_bottom, z_top_range, sur_axe):
    """
    创建一个顶部倾斜的障碍物（类似您代码中的“斜面”）。
    
    Args:
        x_range (tuple): (x_start, x_end)
        y_range (tuple): (y_start, y_end)
        z_bottom (float): 底部平面的z坐标
        z_top_range (tuple): 顶部斜面在x_start和x_end处的z坐标 (z_top_start, z_top_end)

    Returns:
        dict: 包含 'vertices' 和 'faces' 的字典。
    """
    x_start, x_end = x_range
    y_start, y_end = y_range
    z_top_start, z_top_end = z_top_range

    if sur_axe == 'y':
        vertices = [
            # Top face (z varies)
            [x_start, y_end,   z_top_start], # 0
            [x_start, y_start, z_top_start], # 1
            [x_end,   y_start, z_top_end],   # 2
            [x_end,   y_end,   z_top_end],   # 3
            # Bottom face (z is constant)
            [x_start, y_end,   z_bottom],    # 4
            [x_start, y_start, z_bottom],    # 5
            [x_end,   y_start, z_bottom],    # 6
            [x_end,   y_end,   z_bottom]     # 7
        ]
    if sur_axe == 'x':
        vertices = [
            # Top face (z varies)
            [x_end, y_start,   z_top_start], # 0
            [x_start, y_start,    z_top_start], # 1
            [x_start,   y_end, z_top_end],   # 2
            [x_end,   y_end,   z_top_end],   # 3
            # Bottom face (z is constant)
            [x_end, y_start,   z_bottom],    # 4
            [x_start, y_start, z_bottom],    # 5
            [x_start,   y_end, z_bottom],    # 6
            [x_end,   y_end,   z_bottom]     # 7
        ]

    # 面的定义与立方体相同
    faces = [
        [0, 1, 2, 3], [4, 7, 6, 5], [0, 4, 5, 1], 
        [3, 2, 6, 7], [0, 3, 7, 4], [1, 2, 6, 5]
    ]

    return {'vertices': vertices, 'faces': faces}


# --------------------------------------------------
#  请将上面的两个辅助函数 (create_cuboid_obstacle 和 create_sloped_obstacle)
#  粘贴到您的代码文件顶部
# --------------------------------------------------

def get_parameters():
    params = {
        'wg': 2,
        'wf': 1,

        'omega_e': 1,
        'omega_t': 1.2,
        'num_samples': 1000,
        'ground_ratio': 0.4,
        'R_max': 5,
        'e_factor': 0.1,
        'pc': 19,  
        'pf': 946,
        'vc': 0.1,
        'vf': 2.0,
        'm': 2.8,
        'et': 5.0,
        'Wt':600,

        'start': (20, -8, 10),
        'end': (1, 12, 8.1),
        'waypoints': [
            (-3, 2, 6), # 标志物2
            # (7, 0, 10), # 经过点
            (7, 13, 10), # 标志物1
            (1, 12, 8.1), # 斜面目标点
        ],

        'expand_obstacles': [
            # 斜面 
            #create_sloped_obstacle(x_range=(18, 24), y_range=(5, 20), z_bottom=8, z_top_range=(8, 10), sur_axe='x'), 
            
            # 平台
            create_cuboid_obstacle((14, 3, 0), (24, 20, 8)),

            # 起飞平台
            create_cuboid_obstacle((18, -15, 0), (24, -2, 10)),

            # 降落斜面
            create_sloped_obstacle(x_range=(10, -8), y_range=(0, 15), z_bottom=0, z_top_range=(0, 10), sur_axe='x'),
        ],
        
        'connect_obstacles': [
            # 斜面 
            # create_sloped_obstacle(x_range=(18, 24), y_range=(5, 20), z_bottom=0, z_top_range=(8, 10)),

            # 降落斜面
            create_sloped_obstacle(x_range=(10, -8), y_range=(0, 15), z_bottom=0, z_top_range=(0, 10), sur_axe='x'),
            
        ],

        'real_obstacles': [
            # 斜面 
            #create_sloped_obstacle(x_range=(18, 24), y_range=(5, 20), z_bottom=8, z_top_range=(8, 10), sur_axe='x'), 
            
            # 平台
            create_cuboid_obstacle((14, 3, 0), (24, 20, 8)),

            # 起飞平台
            create_cuboid_obstacle((18, -15, 0), (24, -2, 10)),

            # 降落斜面
            create_sloped_obstacle(x_range=(10, -8), y_range=(0, 15), z_bottom=0, z_top_range=(0, 10), sur_axe='x'),
            
        ],
        
        'xlim': (-15, 24),
        'ylim': (-15, 20),
        'zlim': (0, 30),
    }
    return params

# 您可以调用 get_parameters() 并打印其中一个障碍物列表来验证结果
if __name__ == '__main__':
    all_params = get_parameters()
    print("--- Generated expand_obstacles[1] ---")
    import json
    print(json.dumps(all_params['expand_obstacles'][1], indent=4))
    print("\n--- Original expand_obstacles[1] from your code ---")
    original_obstacle = {'vertices': [[3, 16.5, 10], [3, -3.5, 10], [11, -3.5, 10], [11, 16.5, 10],
                                     [3, 16.5, 0], [3, -3.5, 0], [11, -3.5, 0], [11, 16.5, 0]],
                         'faces': [[0,1,2,3], [4,5,6,7], [0,1,5,4], [2,3,7,6],
                                   [0,3,7,4], [1,2,6,5]]}
    # 注：顶点顺序可能不同，但定义的几何体是相同的
    # print(json.dumps(original_obstacle, indent=4))