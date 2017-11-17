from rrt_star import *
from rrg import *
from rrt_nh import *
from rrt_nh import *
from datetime import datetime
from random_data_generator import *
import time
from plotter import *

space_region = ((0, 0, 0), (200, 200, 360))
obstacle = get_random_obstacles(10, space_region)

start = get_random_initial_state(3, space_region, obstacle)
target = get_random_target_state(space_region, obstacle)

# print('Starting RRT*')
# t = datetime.now()
# rrt_star, rrt_star_final_state = apply_rrt_star(space_region=space_region,
#                                                 starting_state=start,
#                                                 target_region=target,
#                                                 obstacle_map=obstacle,
#                                                 d_threshold=10,
#                                                 n_samples=5000,
#                                                 granularity=1)
# print('total time taken: ', datetime.now() - t)

print('Starting RRT')
t = datetime.now()
rrt_nh, rrt_nh_final_state = apply_rrt_nh(space_region=space_region,
                                          starting_state=start,
                                          target_region=target,
                                          obstacle_map=obstacle,
                                          dt=10,
                                          n_samples=500,
                                          granularity=1)
print('total time taken: ', datetime.now() - t)

# plot the trees
# Plotting takes some time as it has to iterate and create each edge. Could not find a better solution
id = int(time.time())
# plot(rrt_star, start, rrt_star_final_state, target, obstacle, 'RRT*', id,is_tree=True)
plot(rrt_nh, start, rrt_nh_final_state, target, obstacle, 'RRT', id, is_tree=False)
