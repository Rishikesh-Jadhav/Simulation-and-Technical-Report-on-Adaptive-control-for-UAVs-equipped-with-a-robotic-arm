"""position_control.py - calculates the thrust and reference angles of the quadrotor."""

import numpy


def calculate_thrust_and_reference_angles(state, controllable_vars_input, gravity, inertia_matrix, coriolis_matrix):

    controllable_vars_input = numpy.array(controllable_vars_input.tolist()).astype(numpy.float64)
    m_p_p = inertia_matrix[0:3, 0:3]
    m_p_phi = inertia_matrix[0:3, 3:6]
    m_p_phi = m_p_phi[:, 0]
    m_p_q = inertia_matrix[0:3, 6:]

    position_control = controllable_vars_input[:3, :]
    yaw_control = controllable_vars_input[3, :]
    manipulator_link_control = controllable_vars_input[4:, :]

    state_deriv = numpy.array([[state.vx], [state.vy], [state.vz], [state.rotational_velocity_yaw],
                               [state.rotational_velocity_pitch], [state.rotational_velocity_roll]]
                              + [[x] for x in state.joint_velocities])

    # TODO - integrate disturbances (d-hat) into this equation too
    u_f = numpy.dot(m_p_p, position_control) + m_p_phi * yaw_control + numpy.dot(m_p_q, manipulator_link_control) + numpy.dot(coriolis_matrix, state_deriv)[:3, :] + gravity[:3, :]

    thrust = numpy.linalg.norm(u_f)

    u_f_x = u_f[0, 0]
    u_f_y = u_f[1, 0]
    u_f_z = u_f[2, 0]
    pitch_reference_angle = numpy.arctan((u_f_x * numpy.cos(state.yaw) + u_f_y * numpy.sin(state.yaw)) / u_f_z)

    roll_reference_angle = numpy.arcsin((u_f_x * numpy.sin(state.yaw) + u_f_y * numpy.cos(state.yaw)) / thrust)

    return thrust, pitch_reference_angle, roll_reference_angle
