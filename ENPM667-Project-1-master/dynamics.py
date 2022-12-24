"""dynamics.py - Contains equations to describe the dynamics of the system"""


import numpy
import jacobian


MOTOR_OFFSET_DISTANCE_M = 0.25

THRUST_COEFFICIENT = 3 * 10**-5
DRAG_COEFFICIENT = 7.5 * 10**-7
THRUST_OVER_DRAG = THRUST_COEFFICIENT / DRAG_COEFFICIENT


def create_base_link_input_forces_from_motor_forces(motor_forces):
    motor_input_to_body_forces = numpy.array([[1, 1, 1, 1],
                                               [0, MOTOR_OFFSET_DISTANCE_M, 0, -MOTOR_OFFSET_DISTANCE_M],
                                               [-MOTOR_OFFSET_DISTANCE_M, 0, MOTOR_OFFSET_DISTANCE_M, 0],
                                               [THRUST_OVER_DRAG, THRUST_OVER_DRAG, THRUST_OVER_DRAG, THRUST_OVER_DRAG]])

    # This matrix contains values for thrust in the z direction (body frame), roll, pitch, and yaw
    body_forces = motor_input_to_body_forces * motor_forces
    return body_forces


def create_motor_forces_from_desired_torque_and_thrust(rotational_torques, thrust):
    desired_forces = numpy.ndarray((4, 1))
    desired_forces[0] = thrust
    print(rotational_torques)
    desired_forces[1:, :] = rotational_torques
    motor_input_to_body_forces = numpy.array([[1, 1, 1, 1],
                                               [0, MOTOR_OFFSET_DISTANCE_M, 0, -MOTOR_OFFSET_DISTANCE_M],
                                               [-MOTOR_OFFSET_DISTANCE_M, 0, MOTOR_OFFSET_DISTANCE_M, 0],
                                               [THRUST_OVER_DRAG, THRUST_OVER_DRAG, THRUST_OVER_DRAG,
                                                THRUST_OVER_DRAG]])

    inv_conversion_matrix = numpy.linalg.inv(motor_input_to_body_forces)
    motor_forces = inv_conversion_matrix * desired_forces
    return motor_forces


GRAVITY = numpy.array([[0], [0], [-9.8]])
LINK_MASSES_G = [80, 10, 6, 2, 1.5]
def calculate_gravity_matrix(joint_positions):
    this_jacobian = jacobian.get_current_jacobian(joint_positions)

    gravity_force = numpy.array()
    for mass in LINK_MASSES_G:
        # this_value
        pass