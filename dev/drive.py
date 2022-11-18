import math

""" To Messure """
FORCE_AT_MAX = 1  # N
MAX_SPEED = 1  # m/s
DISTANCE_TRAVELT_AFTER_MAX_SPEED = 1  # m
TIME_TRAVELT_AFTER_MAX_SPEED = 1  # s
MASS = 1  # kg


""" To Calc """
# First Methode Friction
AIR_FRICTION_COEFFICIENT = FORCE_AT_MAX / MAX_SPEED**2
# F_Applied = m*a = F_Friction = AIR_FRICTION_COEFFICIENT * v²


# Second Methode Friction


# Third Methode Friction
#    -> Simulation


# First Methode Acceletation
ACCELERATION = FORCE_AT_MAX / MASS

# Second Methode from Friction (more as validation)
ACCELERATION_VALIDATION = AIR_FRICTION_COEFFICIENT * MAX_SPEED**2 / MASS
