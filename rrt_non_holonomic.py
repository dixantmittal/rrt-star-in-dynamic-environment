import networkx as nx
from tqdm import tqdm

from commons import *
from control_functions import *
from state_generators import *


def apply_rrt_nh(state_space, starting_state, target_region, fixed_obstacles, dynamic_obstacles=None,
                 dt=0.5, n_samples=1000, find_optimal=True):
    tree = nx.DiGraph()
    tree.add_node(starting_state)

    final_state = None

    min_cost = None

    controls = {}

    fixed_obstacles = add_padding(fixed_obstacles)

    for i in tqdm(range((n_samples))):
        # select node to expand
        m_g, random_point = select_node_to_expand(tree, state_space)

        # sample a new point
        m_new, u = sample_new_point_with_control(m_g, dt, velocity_and_steering_angle,
                                                 new_state_with_v_psi)

        # check if m_new lies in space_region
        if not lies_in_area(m_new, state_space):
            continue

        # check if path between(m_g,m_new) defined by motion-model is collision free
        if not is_collision_free(m_g, m_new, fixed_obstacles, dynamic_obstacles, dt, [u]):
            continue

        # if path is free, add new node to tree
        tree.add_weighted_edges_from([(m_g, m_new, cartesian_distance(np.array(m_g)[:2], np.array(m_new)[:2]))])
        controls[(m_g, m_new)] = u
        if lies_in_area(m_new, target_region):
            print('Target reached at i:', i)
            if min_cost is None:
                final_state = m_new
                if not find_optimal:
                    break
                    # break
            else:
                # if new final state has shorter cost, set it as final state
                cost = nx.shortest_path_length(tree, starting_state, m_new)
                if cost < min_cost:
                    final_state = m_new
                    min_cost = cost

    # Print n_collided
    collided = len(collision_cache.values())
    print('Collided: ', collided, '(', round(collided * 100 / i, 2), '% )')
    if final_state is None:
        print("Target not reached.")
    return tree, final_state, controls
