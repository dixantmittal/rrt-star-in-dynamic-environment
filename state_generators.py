import numpy as np
from parameters import *


# to avoid explosion of state space
def generate_using_velocity_and_steering_angle(current_state, control, dt):
    x, y, theta, t = current_state
    v, psi = control
    theta = np.radians(theta)
    psi = np.radians(psi)

    n_x = x + v * np.cos(theta) * dt
    n_y = y + v * np.sin(theta) * dt
    n_theta = theta + v / CAR_LENGTH * np.tan(psi) * dt

    n_theta = np.degrees(n_theta)
    n_theta = n_theta % 360

    # hack for singularity of theta
    if n_theta > 270:
        n_theta = n_theta - 360

    return (n_x, n_y, n_theta, t + dt)
