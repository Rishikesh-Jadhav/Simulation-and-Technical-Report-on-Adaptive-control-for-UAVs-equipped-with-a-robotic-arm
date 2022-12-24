
import numpy
import gain_constants


def calculate_control_inputs(controllable_vars_accels, state, desired_yaw, desired_vel_yaw,
                             desired_link_positions, desired_link_velocities,
                             desired_position, desired_velocity):
    position_accel = controllable_vars_accels[:3, :]
    yaw_accel = controllable_vars_accels[3, 0]
    link_accel = controllable_vars_accels[4:, :]

    yaw_control = yaw_accel + gain_constants.K_PSI_V * (desired_vel_yaw - state.rotational_velocity_yaw) + \
        gain_constants.K_PSI_P * (desired_yaw - state.yaw)
    q_control = link_accel + numpy.dot(gain_constants.K_Q_V, (desired_link_velocities - numpy.array([[x] for x in state.joint_velocities]))) + \
        numpy.dot(gain_constants.K_Q_P, (desired_link_positions - numpy.array([[x] for x in state.joint_positions])))

    current_velocity = numpy.array([[state.vx], [state.vy], [state.vz]])
    current_position = numpy.array([[state.body_x], [state.body_y], [state.body_z]])
    position_control = position_accel + numpy.dot(gain_constants.K_P_V, (desired_velocity - current_velocity)) + \
        numpy.dot(gain_constants.K_P_P, (desired_position - current_position))

    return position_control, yaw_control, q_control