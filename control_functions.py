import numpy as np
from parameters import *


def velocity_and_steering_angle():
    v_min, v_max = VELOCITY_RANGE
    steer_left_max, steer_right_max = STEERING_RANGE

    if TURN == 'left':
        possible_steer = np.arange(steer_left_max + 1)
        bias = np.ones(int(np.ceil(len(possible_steer) / 2))) * 4
        bias = np.hstack((bias, np.ones(int(np.floor(len(possible_steer) / 2))) * 1))
        bias = bias / np.sum(bias)

    else:
        possible_steer = np.arange(steer_right_max + 1) - steer_right_max
        bias = np.ones(steer_right_max + 1)
        bias = bias / np.sum(bias)

    # sample a velocity
    v = np.random.randn()

    if v >= 0:
        v = v * v_max
    else:
        v = v * v_min

    # sample a discrete steering angle
    steer = np.random.choice(possible_steer, p=bias)

    return (v, steer)
