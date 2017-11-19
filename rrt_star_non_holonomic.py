import networkx as nx
from tqdm import tqdm

from commons import *
from control_functions import *
from state_generators import *


def apply_rrt_star_nh(state_space, starting_state, target_region, fixed_obstacles, dynamic_obstacles=None,
                      dt=0.5, n_samples=1000):
    tree = nx.DiGraph()
    tree.add_node(starting_state)

    n_dim = len(starting_state)
    gamma = 1 + np.power(2, n_dim) * (1 + 1.0 / n_dim) * get_free_area(state_space, fixed_obstacles)

    final_state = None

    min_cost_node = None

    # cost for each vertex
    cost = {starting_state: 0}

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

        # # check if path between(m_g,m_new) defined by motion-model is collision free
        if not is_collision_free(m_g, m_new, fixed_obstacles, dynamic_obstacles, dt, [u]):
            continue

        # find k nearest neighbours
        radius = np.minimum(np.power(gamma / volume_of_unit_ball[n_dim] * np.log(i + 1) / (i + 1),
                                     1 / n_dim), VELOCITY_RANGE[1] * dt)

        m_near = nearest_neighbours(list(tree.nodes), m_new, radius=radius)

        min_cost_node = m_g
        min_cost_d = metric_distance(m_g, m_new)
        min_cost_u = [u]

        # look for shortest cost path to m_new
        for m_g in m_near:

            # find the cost for m_new through m_g
            d = metric_distance(m_g, m_new)
            c = cost[m_g] + d

            # if cost is less than current lowest cost, that means m_new to m_g could be a potential link
            if c < cost[min_cost_node] + min_cost_d:

                # find controls to reach m_new from m_g using single controls and not much deviations
                u = find_controls(m_g, m_new, dt)

                # if control is not feasible
                if u is None:
                    continue

                # check if path between(m_g,m_new) defined by motion-model is collision free
                if not is_collision_free(m_g, m_new, fixed_obstacles, dynamic_obstacles, dt, u):
                    continue

                # if path is free, update the minimum distance
                min_cost_node = m_g
                min_cost_d = d
                min_cost_u = u

        m_g = min_cost_node
        for n in range(len(min_cost_u)):
            m_new = new_state_with_v_psi(m_g, min_cost_u[n], dt)
            d = metric_distance(m_g, m_new)
            tree.add_weighted_edges_from([(m_g, m_new, d)])
            controls[(m_g, m_new)] = min_cost_u[n]
            cost[m_new] = cost[m_g] + d
            m_g = m_new

        m_near = nearest_neighbours(list(tree.nodes), m_new, radius=radius)
        # update m_new's neighbours for paths through m_new
        for m_g in m_near:

            # find the cost for m_g through m_new
            d = metric_distance(m_g, m_new)
            c = cost[m_new] + d

            # if cost is less than current cost, that means m_new to m_g could be a potential link
            if c < cost[m_g]:

                u = find_controls(m_g, m_new, dt)

                # if control is not feasible
                if u is None:
                    continue

                # check if path between(m_g,m_new) is collision free
                if not is_collision_free(m_g, m_new, fixed_obstacles, dynamic_obstacles, dt, u):
                    continue

                tree.remove_edge(list(tree.predecessors(m_g))[0], m_g)
                tree.add_weighted_edges_from([(m_new, m_g, d)])
                cost[m_g] = c
                controls[(m_new, m_g)] = u

        # if target is reached, return the tree and final state
        if lies_in_area(m_new, target_region):
            print('Target reached at i:', i)
            if final_state is None:
                final_state = m_new
            elif cost[m_new] < cost[final_state]:
                final_state = m_new

    # Print n_collided
    collided = len(collision_cache.values())
    print('Collided: ', collided, '(', round(collided * 100 / i, 2), '% )')
    if final_state is None:
        print("Target not reached.")
    return tree, final_state, controls
