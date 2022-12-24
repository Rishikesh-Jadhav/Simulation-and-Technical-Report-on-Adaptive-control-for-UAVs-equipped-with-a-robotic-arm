"""noise_generator.py - generates random noise for the robot state estimation."""


import numpy


JOINT_POSITION_NOISE_MEAN = 10**-4
JOINT_POSITION_NOISE_SIGMA = 5 * 10**-4


def generate_joint_position_noise():
    return numpy.random.normal(JOINT_POSITION_NOISE_MEAN, JOINT_POSITION_NOISE_SIGMA)


ROBOT_POSITION_NOISE_MEAN = 10**-3
ROBOT_POSITION_NOISE_SIGMA = 5 * 10**-3


def generate_robot_position_noise():
    return numpy.random.normal(ROBOT_POSITION_NOISE_MEAN, ROBOT_POSITION_NOISE_SIGMA)


ROBOT_ORIENTATION_NOISE_MEAN = 10**-3
ROBOT_ORIENTATION_NOISE_SIGMA = 10**-3


def generate_robot_orientation_noise():
    return numpy.random.normal(ROBOT_ORIENTATION_NOISE_MEAN, ROBOT_ORIENTATION_NOISE_SIGMA)


ROBOT_ATTITUDE_RATE_NOISE_MEAN = 5 * 10**-3
ROBOT_ATTITUDE_RATE_NOISE_SIGMA = 5 * 10**-3


def generate_robot_attitude_rate_noise():
    return numpy.random.normal(ROBOT_ATTITUDE_RATE_NOISE_MEAN, ROBOT_ATTITUDE_RATE_NOISE_SIGMA)