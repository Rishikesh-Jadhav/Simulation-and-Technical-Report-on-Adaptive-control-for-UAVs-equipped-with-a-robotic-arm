"""test.py - Executes a handful of tests to verify written behavior."""


import numpy
import dynamics


def test_quad_motor_forces():
    thrust = 0
    torques = numpy.zeros((3, 1))
    quad_motor_forces = dynamics.create_motor_forces_from_desired_torque_and_thrust(torques, thrust)
    print("Test 1 output: " + str(quad_motor_forces))


if __name__ == "__main__":
    test_quad_motor_forces()

