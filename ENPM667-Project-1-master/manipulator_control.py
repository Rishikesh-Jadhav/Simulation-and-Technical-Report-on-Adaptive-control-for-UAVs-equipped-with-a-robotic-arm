"""manipulator_control.py - determines the joint output forces based on inverse kinematics results."""


import numpy

def calculate_joint_forces(inertia_matrix, position_control, attitude_control, joint_control, coriolis_matrix, gravity,
                           state):
    m_p_phi_transpose = inertia_matrix[3:6, 0:3]
    m_phi_q_transpose = inertia_matrix[6:, 3:6]
    m_q_q = inertia_matrix[6:, 6:]

    state_deriv_matrix = numpy.ndarray((11, 1))
    state_deriv_matrix[:6, :] = numpy.array([[state.vx],
                                             [state.vy],
                                             [state.vz],
                                             [state.rotational_velocity_yaw],
                                             [state.rotational_velocity_pitch],
                                             [state.rotational_velocity_roll]])
    state_deriv_matrix[6:, :] = numpy.array([[x] for x in state.joint_velocities])

    joint_torques = numpy.dot(m_q_q, joint_control) #+ numpy.dot(coriolis_matrix[6:, :], state_deriv_matrix[6:, :]) + gravity

    return joint_torques
