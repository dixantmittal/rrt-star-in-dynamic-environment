import numpy as np
import pickle
from datetime import datetime
from rrt_non_holonomic import *
from parameters import *
from math import *



# set targets
target = {
    'left': (
        (30, 51, cos(pi), sin(pi + TARGET_OFFSET), T_RANGE[0]),
        (10, 14, cos(pi - TARGET_OFFSET) - cos(pi), sin(pi - TARGET_OFFSET) - sin(pi + TARGET_OFFSET), T_RANGE[1])),
    'right': (
        (60, 35, cos(TARGET_OFFSET), sin(radians(-TARGET_OFFSET)), T_RANGE[0]),
        (10, 14, cos(0) - cos(TARGET_OFFSET), sin(TARGET_OFFSET) - sin(-TARGET_OFFSET),
         T_RANGE[1]))
}

start = (55, 20, 0, 1, 0)

fixed_obstacles = {
    'left_bottom_curb': ((0, 0, -1, -1, 0), (40, 35, 2, 2, T_RANGE[1])),
    'right_bottom_curb': ((60, 0, -1, -1, 0), (40, 35, 2, 2, T_RANGE[1])),
    'left_divider': ((0, 49, -1, -1, 0), (40, 2, 2, 2, T_RANGE[1])),
    'right_divider': ((60, 49, -1, -1, 0), (40, 2, 2, 2, T_RANGE[1])),
    'left_top_curb': ((0, 65, -1, -1, 0), (40, 35, 2, 2, T_RANGE[1])),
    'right_top_curb': ((60, 65, -1, -1, 0), (40, 35, 2, 2, T_RANGE[1])),
}

lane_restrictions = {
    'wrong_lane_1': ((40, 0, -1, -1, 0), (10, 35, 2, 2, T_RANGE[1])),
    'wrong_lane_2': ((0, 35, -1, -1, 0), (40, 14, 2, 2, T_RANGE[1])),
    'wrong_lane_3': ((60, 51, -1, -1, 0), (40, 14, 2, 2, T_RANGE[1])),
    'wrong_lane_4': ((40, 65, -1, -1, 0), (10, 35, 2, 2, T_RANGE[1]))
}

moving_obstacles = pickle.load(open('cars.pkl', 'rb'))
dynamic_obstacles = {}
# build dynamic obstacle map
for car_pos in moving_obstacles.values():
    for x, y, d, t in car_pos:
        t = round(t, 1)
        if dynamic_obstacles.get(t) is None:
            dynamic_obstacles[t] = [(x, y, d)]
        else:
            dynamic_obstacles[t].append((x, y, d))

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    space_region = ((0, 0, -1, -1, T_RANGE[0]), (SPACE_DIMS[0], SPACE_DIMS[1], 2, 2, T_RANGE[1]))
    print('Starting RRT')
    t = datetime.now()
    rrt_nh, rrt_nh_final_state, controls = apply_rrt_nh(space_region=space_region,
                                                        starting_state=start,
                                                        target_region=target[TURN],
                                                        fixed_obstacles={**fixed_obstacles, **lane_restrictions},
                                                        dynamic_obstacles=dynamic_obstacles,
                                                        dt=0.2,
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
        patch = patches.Rectangle(car[10, 0:2] - 1, CAR_DIMS[0], CAR_DIMS[1], linewidth=1, edgecolor='r', facecolor='r')
        ax.add_patch(patch)

    # target region
    for tgt in target.values():
        patch = patches.Rectangle((tgt[0][0], tgt[0][1]), tgt[1][0], tgt[1][1], linewidth=1, edgecolor='g',
                                  facecolor='w')
        ax.add_patch(patch)

    # start state
    patch = patches.Rectangle((start[0] - 1, start[1] - 2), CAR_DIMS[1], CAR_DIMS[0], linewidth=1, edgecolor='g',
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

    plt.ylim(Y_RANGE)
    plt.xlim(X_RANGE)
    plt.show()
