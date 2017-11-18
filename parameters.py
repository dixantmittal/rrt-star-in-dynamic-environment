from math import *

# car length in meters
CAR_LENGTH = 5

ACCELERATION_RANGE = (-20, 10)
VELOCITY_RANGE = (-5, 10)

# steering angle (left_max,right_max)
STEERING_RANGE = (45, 45)

OBSTACLE_RANGE_FACTOR = (10, 10, 360)
TARGET_RANGE_FACTOR = (10, 10, 360)

SPACE_DIMS = (100, 100)
CAR_DIMS = (4, 2)
CAR_AXIS = (2, 1)

# NOTE: state is defined by 5 variables (x,y,cos(theta),sin(theta),t)

# set ranges for variables
X_RANGE = (0, SPACE_DIMS[0])
Y_RANGE = (0, SPACE_DIMS[1])
T_RANGE = (0, 200)

# difference in desired orientation allowed for target
TARGET_OFFSET = radians(5)

TURN = 'left'