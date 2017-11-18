import numpy as np
from utils import *


def select_node_to_expand(tree, space_region):
    space_region = np.asarray(space_region)
    space_origin, space_range = space_region
    n_dim = len(space_origin)
    random_point = np.random.rand(n_dim) * (space_range)
    nodes = list(tree.nodes())
    d = cartesian_distance(nodes, random_point)
    return nodes[np.asscalar(np.argmin(d))], random_point


def sample_new_point_with_control(m_g, dt, control_function, state_generator):
    controls = control_function()
    m_new = state_generator(m_g, controls, dt)
    return tuple(m_new)
