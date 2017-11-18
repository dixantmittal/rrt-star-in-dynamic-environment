import numpy as np
from utils import *


def select_node_to_expand(tree, state_space):
    state_space = np.asarray(state_space)
    space_origin, space_range = state_space
    n_dim = len(space_origin)

    # sample a random point in the space
    random_point = np.random.rand(n_dim) * space_range

    # calculate the distance from random point to all nodes
    nodes = list(tree.nodes())
    d = cartesian_distance(nodes, random_point)

    # return the node with shortest distance
    return nodes[np.argmin(d)], random_point


def sample_new_point_with_control(m_g, dt, control_function, state_generator):
    # sample controls
    controls = control_function()

    # calculate new state
    m_new = state_generator(m_g, controls, dt)

    return tuple((tuple(m_new), controls))
