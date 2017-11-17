import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import networkx as nx


def plot(graph, start_state, end_state, target_region, obstacle_map, title, id, is_tree=True):
    nodes = np.asarray(list(graph.nodes))
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal')
    target_region = np.array(target_region)
    target_rect = patches.Rectangle(target_region[0, :2], target_region[1][0], target_region[1][1], linewidth=1,
                                    edgecolor='g', facecolor='g')
    ax.add_patch(target_rect)

    for val in obstacle_map.values():
        val=np.asarray(val)
        ax.add_patch(patches.Rectangle(val[0, :2], val[1][0], val[1][1], linewidth=1, edgecolor='r', facecolor='r'))

    if is_tree:
        edges = list(graph.edges)
        for edge in edges:
            edge = np.array(edge).transpose()
            plt.plot(edge[0, :2], edge[1, :2], 'c-', edge[0], edge[1], 'bo', ms=1,label='Sampled Points')
    else:
        plt.plot(nodes[:, 0], nodes[:, 1], 'bo', ms=1, label='Sampled Points')

    if end_state is not None:
        path = nx.shortest_path(graph, start_state, end_state)
        plt.plot(np.array(path)[:, 0], np.array(path)[:, 1], 'k-', ms=5, label='Returned Path')
    plt.title(title)
    plt.legend()
    plt.show()
    # plt.savefig('plots/' + str(id) + '_' + title + '.png')
