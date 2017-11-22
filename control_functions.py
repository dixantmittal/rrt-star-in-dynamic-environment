import numpy as np

from parameters import *

####################-------------INITIALIZATION-------------####################
# init one time initialized common variables to improve speed while running
from state_generators import new_state_with_v_psi

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


def find_controls(_from, _to, dt):
    x1, y1, c1, s1, t1 = _from
    x2, y2, c2, s2, t2 = _to
    u = []
    for i in range(int(round(t2 - t1, 1) / dt)):
        x1, y1, c1, s1, t1 = _from
        x2, y2, c2, s2, t2 = _to

        theta1 = atan2(s1, c1)
        theta2 = atan2(s2, c2)

        dtheta = theta2 - theta1
        dx = x2 - x1
        dy = y2 - y1
        delta_t = t2 - t1

        if delta_t <= 0:
            return None

        v = dx / (delta_t * c1 + 1e-10)

        if not v_min <= v <= v_max or v < 0:
            return None

        steer_max, steer_min = STEERING_RANGE
        steer_min = cos(-steer_min)
        steer_max = cos(steer_max)

        steer_range = max(steer_max, steer_min)

        psi = atan2(dtheta * CAR_LENGTH * c1, dx)
        if cos(psi) < steer_range:
            return None
        control = (v, degrees(psi))
        u.append(control)
        _from = new_state_with_v_psi(_from, control, dt)

    if len(u) == 0:
        return None
    return u
