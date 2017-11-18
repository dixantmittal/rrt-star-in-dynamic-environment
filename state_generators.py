import numpy as np
from parameters import *


# NOTE: theta and psi angles are required to be in radians
def generate_using_velocity_and_steering_angle(current_state, control, dt):
    x, y, theta, t = current_state
    v, psi = control

    # convert angles to radians for calculations
    theta = np.radians(theta)
    psi = np.radians(psi)

    # calculate new state
    n_x = x + v * np.cos(theta) * dt
    n_y = y + v * np.sin(theta) * dt
    n_theta = theta + v / CAR_LENGTH * np.tan(psi) * dt
    n_t = t + dt

    # convert angles back to radians
    n_theta = np.degrees(n_theta)
    n_theta = n_theta % 360

    return tuple((n_x, n_y, n_theta, n_t))
