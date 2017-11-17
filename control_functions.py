import numpy as np
from parameters import *


def acceleration_and_steering_angle():
    min_acc, max_acc = ACCELERATION_RANGE
    min_steer, max_steer = STEERING_RANGE

    acc = np.random.randn()

    if acc >= 0:
        acc = acc * max_acc
    else:
        acc = acc * min_acc

    steer = np.random.rand()

    if steer >= 0:
        steer = steer * max_steer
    else:
        steer = steer * min_steer

    return (acc, steer)


def velocity_and_steering_angle():
    v_min, v_max = VELOCITY_RANGE
    steer_min, steer_max = STEERING_RANGE

    # sample a velocity
    v = np.random.randn()

    if v >= 0:
        v = v * v_max
    else:
        v = v * v_min

    # sample a steering angle
    steer = np.random.rand()

    if steer >= 0:
        steer = steer * steer_max
    else:
        steer = steer * steer_min

    return (v, steer)
