import numpy as np
from parameters import *
from math import *


# NOTE: theta and psi angles are required to be in radians
def new_state_with_v_psi(current_state, control, dt):
    x, y, cos_theta, sin_theta, t = current_state
    v, psi = control

    # convert angles to radians for calculations
    theta = atan2(sin_theta, cos_theta)
    psi = np.radians(psi)

    # calculate new state
    n_x = x + v * cos_theta * dt
    n_y = y + v * sin_theta * dt
    n_theta = theta + v / CAR_LENGTH * np.tan(psi) * dt
    n_t = t + dt

    # create angle states
    n_cos_theta = cos(n_theta)
    n_sin_theta = sin(n_theta)

    return tuple((n_x, n_y, n_cos_theta, n_sin_theta, n_t))
