"""inverse_kinematics.py - Calculates the desired behavior of the controllable variables."""


import jacobian
import gain_constants
import transformation_matrices
import numpy
import math


def generate_controllable_variable_accelerations(state, end_effector_position, desired_ee_position,
                                                 desired_ee_rotation, desired_ee_velocity, desired_ee_accel):
    controlled_vars_deriv = numpy.ndarray((9, 1))
    controlled_vars_deriv[0:4, :] = numpy.array([[state.vx], [state.vy], [state.vz], [state.rotational_velocity_yaw]])
    controlled_vars_deriv[4:, :] = numpy.array([[x] for x in state.joint_velocities])

    uncontrolled_vars_deriv = numpy.array([[state.rotational_velocity_pitch], [state.rotational_velocity_roll]])
    uncontrolled_vars_accel = numpy.array([[state.rot_accel_pitch], [state.rot_accel_roll]])

    # The jacobian of controllable variables
    controllable_jac = jacobian.get_jacobian_of_controllable_variables(state.joint_positions, state, end_effector_position)

    pseudo_inv_controllable_jac = numpy.linalg.pinv(controllable_jac)
    # pseudo_inv_controllable_jac = numpy.dot(controllable_jac, controllable_jac.transpose())
    # pseudo_inv_controllable_jac = controllable_jac.transpose() * numpy.linalg.inv(pseudo_inv_controllable_jac)

    derivative_of_controlled_jac = jacobian.get_deriv_of_controlled_jacobian(state.joint_positions, state.joint_velocities,
                                                                             state, end_effector_position)

    uncontrollable_jac = jacobian.get_jacobian_of_uncontrolled_variables(state, end_effector_position)
    derivative_of_unc_jac = uncontrollable_jac # TODO - replace this with the derivative

    e = _calculate_ee_error(desired_ee_position, state, desired_ee_rotation)

    joint_velocities = numpy.array([[x] for x in state.joint_velocities])
    ee_velocity = jacobian.get_current_jacobian(state.joint_positions) * joint_velocities
    desired_ee_behavior = numpy.ndarray((6, 1))
    desired_ee_behavior[:3, :] = desired_ee_velocity
    desired_ee_behavior[3:, :] = desired_ee_rotation
    ee_velocity_error = desired_ee_behavior - ee_velocity

    desired_ee_accel2 = numpy.zeros((6, 1))
    desired_ee_accel2[:3, :] = desired_ee_accel
    controllable_accelerations = pseudo_inv_controllable_jac * (desired_ee_accel2 + gain_constants.K_V * ee_velocity_error + numpy.dot(gain_constants.K_P, e)) - \
        numpy.dot(pseudo_inv_controllable_jac, (numpy.dot(derivative_of_controlled_jac, controlled_vars_deriv) + numpy.dot(uncontrollable_jac, uncontrolled_vars_accel) + numpy.dot(derivative_of_unc_jac, uncontrolled_vars_deriv)))

    return controllable_accelerations


def _calculate_ee_error(desired_ee_position, state, desired_ee_rotation):
    e = numpy.ndarray((6, 1))
    e[0:3, :] = desired_ee_position - transformation_matrices.get_ee_position(state)

    # Get YPR of the end effector
    ee_rot_matrix = jacobian.T0_n[-1][:3, :3]
    ee_rot_matrix = ee_rot_matrix.subs([(jacobian.q1, state.joint_positions[0]),
                                        (jacobian.q2, state.joint_positions[1]),
                                        (jacobian.q3, state.joint_positions[2]),
                                        (jacobian.q4, state.joint_positions[3]),
                                        (jacobian.q5, state.joint_positions[4])])
    ee_pitch = math.atan2(-ee_rot_matrix[2, 0], math.sqrt(ee_rot_matrix[2, 1]**2 + ee_rot_matrix[2, 2]**2))
    ee_yaw = math.atan2(ee_rot_matrix[2, 1] / math.cos(ee_pitch), ee_rot_matrix[2, 2] / math.cos(ee_pitch))
    ee_roll = math.atan2(ee_rot_matrix[1, 0] / math.cos(ee_pitch), ee_rot_matrix[0, 0] / math.cos(ee_pitch))
    current_ee_rotation = numpy.array([[ee_yaw], [ee_pitch], [ee_roll]])

    mutual_orientation_matrix = desired_ee_rotation.transpose() * current_ee_rotation

    # Convert to quaternion
    yaw = mutual_orientation_matrix[0, 0]
    pitch = mutual_orientation_matrix[0, 1]
    roll = mutual_orientation_matrix[0, 2]
    qx = numpy.sin(roll / 2) * numpy.cos(pitch / 2) * numpy.cos(yaw / 2) - numpy.cos(roll / 2) * numpy.sin(
        pitch / 2) * numpy.sin(yaw / 2)
    qy = numpy.cos(roll / 2) * numpy.sin(pitch / 2) * numpy.cos(yaw / 2) + numpy.sin(roll / 2) * numpy.cos(
        pitch / 2) * numpy.sin(yaw / 2)
    qz = numpy.cos(roll / 2) * numpy.cos(pitch / 2) * numpy.sin(yaw / 2) - numpy.sin(roll / 2) * numpy.sin(
        pitch / 2) * numpy.cos(yaw / 2)
    normalize_factor = math.sqrt(qx**2 + qy**2 + qz**2)
    if normalize_factor != 0:
        qx /= normalize_factor
        qy /= normalize_factor
        qz /= normalize_factor

    e[3:6, :] = desired_ee_position * numpy.array([[qx], [qy], [qz]])

    return e
