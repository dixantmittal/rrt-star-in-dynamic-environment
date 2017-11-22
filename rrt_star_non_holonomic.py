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

    for i in tqdm(range(n_samples)):

        # update cost cache
        if i % 500 == 0:
            cost = nx.single_source_dijkstra_path_length(tree, starting_state)

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

        # find k nearest neighbours
        radius = np.minimum(np.power(gamma / volume_of_unit_ball[2] * np.log(i + 1) / (i + 1),
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
            # due to minute difference in final states, it is better to force last state as actual final state
            # force last node to be actual target node.
            if n == len(min_cost_u) - 1:
                m_inter = m_new
            else:
                m_inter = new_state_with_v_psi(m_g, min_cost_u[n], dt)

            d = metric_distance(m_g, m_inter)
            tree.add_weighted_edges_from([(m_g, m_inter, d)])
            controls[(m_g, m_inter)] = min_cost_u[n]
            cost[m_inter] = cost[m_g] + d
            m_g = m_inter

        # update m_new's neighbours for paths through m_new
        for m_g in m_near:

            # find the cost for m_g through m_new
            d = metric_distance(m_g, m_new)
            c = cost[m_new] + d

            # if cost is less than current cost, that means m_new to m_g could be a potential link
            if c < cost[m_g]:

                u = find_controls(m_new, m_g, dt)

                # if control is not feasible
                if u is None:
                    continue

                # check if path between(m_g,m_new) is collision free
                if not is_collision_free(m_new, m_g, fixed_obstacles, dynamic_obstacles, dt, u):
                    continue

                tree.remove_edge(list(tree.predecessors(m_g))[0], m_g)
                m_inter1 = m_new
                for n in range(len(u)):
                    if n == len(u) - 1:
                        m_inter2 = m_g
                    else:
                        m_inter2 = new_state_with_v_psi(m_inter1, u[n], dt)

                    d = metric_distance(m_inter1, m_inter2)
                    tree.add_weighted_edges_from([(m_inter1, m_inter2, d)])
                    controls[(m_inter1, m_inter2)] = u[n]
                    cost[m_inter2] = cost[m_inter1] + d
                    m_inter1 = m_inter2

        # if target is reached, return the tree and final state
        if lies_in_area(m_new, target_region):
            cost = nx.single_source_dijkstra_path_length(tree, starting_state)
            print('Target reached at i:', i)
            if final_state is None:
                final_state = m_new
            elif cost[m_new] < cost[final_state]:
                final_state = m_new

    if final_state is None:
        print("Target not reached.")
    return tree, final_state, controls
