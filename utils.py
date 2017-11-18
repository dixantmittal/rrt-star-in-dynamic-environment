import numpy as np
from parameters import *

volume_of_unit_ball = {
    1: 2,
    2: 3.142,
    3: 4.189,
    4: 4.935,
    5: 5.264,
    6: 5.168,
    7: 4.725,
    8: 4.059,
    9: 3.299,
    10: 2.550
}

collision_cache = {}

free_space_cache = {}


def get_free_area(space_region, obstacle_map):
    _, space_range = space_region
    l, b = space_range
    space_area = l * b

    obstacle_area = 0
    for obstacle in obstacle_map.values():
        _, obstacle_range = obstacle
        l, b = obstacle_range
        obstacle_area += l * b

    return space_area - obstacle_area


def lies_in_area(point, area):
    frame, _range = area
    frame = np.array(frame)
    point = np.array(point)

    diff = point - frame

    return np.all(diff <= _range) and np.all(diff >= 0)


def nearest_neighbours(nodes, center, radius):
    nodes = np.asarray(nodes)
    d = cartesian_distance(nodes, center)
    nearest_nodes = nodes[d < radius]
    return tuple(map(tuple, nearest_nodes))


def cartesian_distance(x, y):
    x = np.array(x)
    y = np.array(y)

    if x.ndim == 1:
        x = x.reshape(1, -1)

    if y.ndim == 1:
        y = y.reshape(1, -1)

    dist = np.sqrt(np.sum((y - x) ** 2, axis=1))
    return dist


def is_fixed_obstacle_space(point, obstacle_map):
    if obstacle_map is None:
        return False

    for key in obstacle_map.keys():
        if lies_in_area(point, obstacle_map[key]):
            return True
    return False


def is_dynamic_obstacle_space(y, obstacle_map):
    if obstacle_map is None:
        return False

    # for key in obstacle_map.keys():
    #     if lies_in_area(point, obstacle_map[key]):
    #         return True
    return False


def is_collision_free(x, y, fixed_obstacles, dynamic_obstacles):
    if collision_cache.get(y, False):
        return False

    if is_fixed_obstacle_space(y, fixed_obstacles):
        collision_cache[y] = True
        return False

    if is_dynamic_obstacle_space(y, dynamic_obstacles):
        collision_cache[y] = True
        return False

    return True


def add_padding(obstacle_map):
    max_dim = np.max(CAR_AXIS)
    for key in obstacle_map.keys():
        (x, y, c, s, t), (x_range, y_range, c_range, s_range, t_range) = obstacle_map[key]
        x = x - max_dim
        y = y - max_dim

        x_range = x_range + 2 * max_dim
        y_range = y_range + 2 * max_dim

        obstacle_map[key] = ((x, y, c, s, t), (x_range, y_range, c_range, y_range, t_range))

    return obstacle_map
