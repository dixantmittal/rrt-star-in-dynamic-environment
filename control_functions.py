import numpy as np
from parameters import *

####################-------------INITIALIZATION-------------####################
# init one time initialized common variables to improve speed while running
v_min, v_max = VELOCITY_RANGE
steer_left_max, steer_right_max = STEERING_RANGE

# for velocity
POSSIBLE_V = np.arange(v_max - v_min + 1) + v_min  # v_max, v_min are inclusive
# extreme velocities are favoured
# v_bias = possible_v ** 4
# uniform
# v_bias = np.ones(possible_v.shape)
# forward velocities are favoured
V_BIAS = np.arange(POSSIBLE_V.shape[0]) ** 2 + 1
V_BIAS = V_BIAS / np.sum(V_BIAS)  # normalize

# for steering angle
if TURN == 'left':
    POSSIBLE_STEER = np.arange(steer_left_max + 1)

    # steer_bias = np.ones(int(np.ceil(len(possible_steer) / 2))) * 4
    # steer_bias = np.hstack((steer_bias, np.ones(int(np.floor(len(possible_steer) / 2))) * 1))

    # favours straight steering by squared factors
    STEER_BIAS = np.flip(np.arange(POSSIBLE_STEER.shape[0]), axis=0) ** 2
    STEER_BIAS = STEER_BIAS / np.sum(STEER_BIAS)

else:
    POSSIBLE_STEER = -np.arange(steer_right_max + 1)

    # uniform samplings
    STEER_BIAS = np.ones(steer_right_max + 1)

    # favours turning right bu squared factor
    # steer_bias = np.arange(possible_steer.shape[0])

    STEER_BIAS = STEER_BIAS / np.sum(STEER_BIAS)


####################------------------------------------------####################



def velocity_and_steering_angle():
    steer = sample_steering_angle()
    v = sample_velocity()
    return (v, steer)


def sample_velocity():
    # sample a discrete velocity
    return np.random.choice(POSSIBLE_V, p=V_BIAS)


def sample_steering_angle():
    return np.random.choice(POSSIBLE_STEER, p=STEER_BIAS)
