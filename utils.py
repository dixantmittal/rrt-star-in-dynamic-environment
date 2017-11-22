from state_generators import *

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
    l, b, c, s, t = space_range
    space_area = l * b

    obstacle_area = 0
    for obstacle in obstacle_map.values():
        _, obstacle_range = obstacle
        l, b, c, s, t = obstacle_range
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
    center = np.asarray(center)
    d = cartesian_distance(nodes[:, :2], center[:2])
    nearest_nodes = nodes[d <= radius]
    return tuple(map(tuple, nearest_nodes))


def metric_distance(x, y):
    x = np.array(x)[:2]
    y = np.array(y)[:2]

    return cartesian_distance(x, y)


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


def is_dynamic_obstacle_space(point, obstacle_map, dt):
    max_dim = np.max(CAR_AXIS)
    if obstacle_map is None:
        return False

    t = point[4]

    cars_before = obstacle_map.get(round(t - dt, 1), [])
    cars_after = obstacle_map.get(round(t + dt, 1), [])

    # for simplicity, we will take position of car at t-dt,t,d+dt and treat complete area as obstacle
    for i in range(len(cars_after)):
        x1, y, theta = cars_before[i]
        x2 = cars_after[i][0]

        if x1 > x2:
            temp = x2
            x2 = x1
            x1 = temp

        region = ((x1 - max_dim, y - max_dim, t - dt), (x2 + max_dim - x1 + max_dim, 2 * max_dim, t + dt))

        if lies_in_area(np.array(point)[[0, 1, 4]], region):
            return True

    return False


def is_collision_free(x, y, fixed_obstacles, dynamic_obstacles, dt, controls=None):
    for i in range(len(controls)):
        x = new_state_with_v_psi(x, controls[i], dt)

        if collision_cache.get(x, False):
            return False

        if is_fixed_obstacle_space(x, fixed_obstacles):
            collision_cache[x] = True
            return False

        if is_dynamic_obstacle_space(x, dynamic_obstacles, dt):
            collision_cache[x] = True
            return False
    return True

def grow_obstacle(obstacle_region):
    max_dim = np.max(CAR_AXIS)
    (x, y, c, s, t), (x_range, y_range, c_range, s_range, t_range) = obstacle_region
    x = x - max_dim
    y = y - max_dim

    x_range = x_range + 2 * max_dim
    y_range = y_range + 2 * max_dim

    return ((x, y, c, s, t), (x_range, y_range, c_range, s_range, t_range))


def add_padding(obstacle_map):
    for key in obstacle_map.keys():
        obstacle_map[key] = grow_obstacle(obstacle_map[key])

    return obstacle_map
