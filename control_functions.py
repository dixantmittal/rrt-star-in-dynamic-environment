import numpy as np
from parameters import *

####################-------------INITIALIZATION-------------####################
# init one time initialized common variables to improve speed while running
v_min, v_max = VELOCITY_RANGE
steer_left_max, steer_right_max = STEERING_RANGE

# for velocity
POSSIBLE_V = np.arange(v_max - v_min + 1) + v_min  # v_max, v_min are inclusive

# forward velocities are favoured
V_BIAS = np.array(POSSIBLE_V)
V_BIAS[V_BIAS < 1] = 1
V_BIAS = V_BIAS / np.sum(V_BIAS)  # normalize

# for steering angle
POSSIBLE_STEER = -np.arange(steer_right_max + steer_left_max + 1) + steer_left_max

if TURN == 'left':
    # favouring left steer with factor 2 to 3
    STEER_BIAS = np.array(POSSIBLE_STEER)
elif TURN == 'right':
    # favouring right steer with factor 2 to 3
    STEER_BIAS = np.array(-POSSIBLE_STEER)
else:
    # favouring central angles more
    STEER_BIAS = 4 / (np.abs(POSSIBLE_STEER) + 0.1)

STEER_BIAS[STEER_BIAS < 1] = 1
STEER_BIAS = np.sqrt(STEER_BIAS)
STEER_BIAS[STEER_BIAS > 3] = 3
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
