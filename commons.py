import numpy as np
from utils import *


def select_node_to_expand(tree, state_space):
    state_space = np.asarray(state_space)
    space_origin, space_range = state_space
    n_dim = len(space_origin)

    # sample a random point in the space
    random_point = np.random.rand(2) * space_range[0:2]
    theta = np.random.rand() * 2 * np.pi
    random_point = np.hstack((random_point, (np.cos(theta), np.sin(theta))))

    # calculate the distance from random point to all nodes, excluding time dimension
    nodes = list(tree.nodes())
    d = cartesian_distance(np.array(nodes)[:, 0:4], random_point)

    # return the node with shortest distance
    return nodes[np.argmin(d)], random_point


def sample_new_point_with_control(m_g, dt, control_function, state_generator):
    # sample controls
    controls = control_function()

    # calculate new state
    m_new = state_generator(m_g, controls, dt)

    return tuple((tuple(m_new), controls))
