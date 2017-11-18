import numpy as np
import pickle
from datetime import datetime
from rrt_non_holonomic import *
from parameters import *
from math import *

space_dims = (100, 100)
car_dims = (4, 2)
car_axis = (2, 1)

# NOTE: state is defined by 5 variables (x,y,cos(theta),sin(theta),t)

# set ranges for variables
x_range = (0, space_dims[0])
y_range = (0, space_dims[1])
t_range = (0, 200)

# difference in desired orientation allowed for target
target_offset = radians(5)

# set targets
target = {
    'left': (
        (30, 51, cos(pi), sin(pi + target_offset), t_range[0]),
        (10, 14, cos(pi - target_offset) - cos(pi), sin(pi - target_offset) - sin(pi + target_offset), t_range[1])),
    'right': (
        (60, 35, cos(radians(-target_offset)), sin(radians(-target_offset)), t_range[0]),
        (10, 14, cos(0), sin(radians(target_offset)) - sin(radians(-target_offset)), t_range[1]))
}

start = (55, 20, 0, 1, 0)

fixed_obstacles = {
    'left_bottom_curb': ((0, 0, -1, -1, 0), (40, 35, 2, 2, t_range[1])),
    'right_bottom_curb': ((60, 0, -1, -1, 0), (40, 35, 2, 2, t_range[1])),
    'left_divider': ((0, 49, -1, -1, 0), (40, 2, 2, 2, t_range[1])),
    'right_divider': ((60, 49, -1, -1, 0), (40, 2, 2, 2, t_range[1])),
    'left_top_curb': ((0, 65, -1, -1, 0), (40, 35, 2, 2, t_range[1])),
    'right_top_curb': ((60, 65, -1, -1, 0), (40, 35, 2, 2, t_range[1])),
}

lane_restrictions = {
    'wrong_lane_1': ((40, 0, -1, -1, 0), (10, 35, 2, 2, t_range[1])),
    'wrong_lane_2': ((0, 35, -1, -1, 0), (40, 14, 2, 2, t_range[1])),
    'wrong_lane_3': ((60, 51, -1, -1, 0), (40, 14, 2, 2, t_range[1])),
    'wrong_lane_4': ((40, 65, -1, -1, 0), (10, 35, 2, 2, t_range[1]))
}

moving_obstacles = pickle.load(open('cars.pkl', 'rb'))

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    space_region = ((0, 0, -1, -1, t_range[0]), (space_dims[0], space_dims[1], 2, 2, t_range[1]))
    print('Starting RRT')
    t = datetime.now()
    rrt_nh, rrt_nh_final_state, controls = apply_rrt_nh(space_region=space_region,
                                                        starting_state=start,
                                                        target_region=target[TURN],
                                                        obstacle_map={**fixed_obstacles, **lane_restrictions},
                                                        # {**fixed_obstacles, **moving_obstacles},
                                                        dt=0.1,
                                                        n_samples=5000,
                                                        granularity=10000000)
    print('total time taken: ', datetime.now() - t)

    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')
    for obstacle in fixed_obstacles.values():
        patch = patches.Rectangle((obstacle[0][0], obstacle[0][1]), obstacle[1][0], obstacle[1][1], linewidth=1,
                                  edgecolor='0.5', facecolor='0.5')
        ax.add_patch(patch)

    for car in moving_obstacles.values():
        patch = patches.Rectangle(car[30, 0:2] - 1, car_dims[0], car_dims[1], linewidth=1, edgecolor='r', facecolor='r')
        ax.add_patch(patch)

    # target region
    for tgt in target.values():
        patch = patches.Rectangle((tgt[0][0], tgt[0][1]), tgt[1][0], tgt[1][1], linewidth=1, edgecolor='g',
                                  facecolor='w')
        ax.add_patch(patch)

    # start state
    patch = patches.Rectangle((start[0] - 1, start[1]-2), car_dims[1], car_dims[0], linewidth=1, edgecolor='g',
                              facecolor='g')
    ax.add_patch(patch)

    nodes = np.asarray(list(rrt_nh.nodes))
    plt.plot(nodes[:, 0], nodes[:, 1], 'bo', ms=1, label='Sampled Points')

    if rrt_nh_final_state is not None:
        path = nx.shortest_path(rrt_nh, start, rrt_nh_final_state)
        plt.plot(np.array(path)[:, 0], np.array(path)[:, 1], 'k-', ms=5, label='Returned Path')

        # print controls
        for i in range(len(path) - 1):
            print(controls[(path[i], path[i + 1])])

    plt.ylim(y_range)
    plt.xlim(x_range)
    plt.show()
