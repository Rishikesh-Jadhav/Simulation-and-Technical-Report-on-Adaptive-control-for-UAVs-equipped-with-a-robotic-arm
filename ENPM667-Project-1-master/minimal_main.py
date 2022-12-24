"""minimal_main.py - Runs the minimum amount of code needed to close the loop and generate 60 seconds' worth of data."""


import math
import numpy
import transformation_matrices
import system_state
import reference_trajectory
import inverse_kinematics
import position_control
import attitude_control
import motion_control
import dynamics
import manipulator_control
import system_emulator
import visualization

MAX_ACC = 3
TIME_RESOLUTION_S = 1 / 10 #1 / 250
END_TIME_S = 60


def main():

    state = system_state.SystemState()

    t = 0
    while t <= END_TIME_S:
        print(t)

        try:
            desired_pos, desired_vel, desired_accel, desired_rot = reference_trajectory.generate_reference_trajectory(t)
            ee_position = transformation_matrices.get_ee_position(state)
            print(ee_position)
            controllable_vars = inverse_kinematics.generate_controllable_variable_accelerations(state, ee_position, desired_pos,
                                                                                                desired_rot, desired_vel, desired_accel)

            # Normalize positional acceleration
            x_acc = controllable_vars[0, 0]
            y_acc = controllable_vars[1, 0]
            z_acc = controllable_vars[2, 0]
            acc_norm = MAX_ACC * math.sqrt(x_acc**2 + y_acc**2 + z_acc**2)
            if acc_norm != 0:
                controllable_vars[0, 0] /= acc_norm
                controllable_vars[1, 0] /= acc_norm
                controllable_vars[2, 0] /= acc_norm

            # Normalize arm commands
            joint_norm = 0
            for i in range(len(state.joint_positions)):
                joint_norm += controllable_vars[4+i, 0]**2
            joint_norm = 2 * math.sqrt(joint_norm)
            if joint_norm != 0:
                for i in range(len(state.joint_positions)):
                    controllable_vars[4+i, 0] /= joint_norm

            print("Controllable accelerations: " + str(controllable_vars))

            # Integrate reference yaw acceleration to get desired yaw_vel and yaw
            # desired_yaw_accel = controllable_vars[4, 0]
            # desired_yaw_vel = state.rotational_velocity_yaw + desired_yaw_accel * TIME_RESOLUTION_S
            # desired_yaw = state.yaw + desired_yaw_vel * TIME_RESOLUTION_S
            #
            # # Do the same with each manipulator link
            # desired_link_positions = []
            # desired_link_vels = []
            # for i in range(len(state.joint_positions)):
            #     desired_link_accel = controllable_vars[4+i, 0]
            #     desired_link_vel = state.joint_velocities[i] + desired_link_accel * TIME_RESOLUTION_S
            #     desired_link_position = state.joint_positions[i] + desired_link_vel * TIME_RESOLUTION_S
            #     desired_link_positions.append([desired_link_position])
            #     desired_link_vels.append([desired_link_vel])
            # desired_link_vels = numpy.array(desired_link_vels)
            # desired_link_positions = numpy.array(desired_link_positions)

            # pos_control_input, yaw_control_input, link_control_input = motion_control.calculate_control_inputs(controllable_vars,
            #                                                                                                    state,
            #                                                                                                    desired_yaw,
            #                                                                                                    desired_yaw_vel,
            #                                                                                                    desired_link_positions,
            #                                                                                                    desired_link_vels,
            #                                                                                                    desired_pos,
            #                                                                                                    desired_vel)

            # thrust, pitch, roll = position_control.calculate_thrust_and_reference_angles(state, controllable_vars, gravity,
            #                                                            p                  inertia_matrix, coriolis_matrix)
            #
            # pitch_control, roll_control = attitude_control.generate_attitude_control_input_estimates(roll, roll_deriv,
            #                                                                                          pitch, pitch_deriv,
            #                                                                                          state)
            #
            # rotational_control_input = numpy.array([[yaw_control_input], [pitch_control], [roll_control]])
            # rotational_control_input = numpy.array([[yaw_control_input], [0], [0]])
            # vehicle_torques = attitude_control.calculate_vehicle_torques(inertia_matrix, pos_control_input, rotational_control_input,
            #                                            link_control_input, g, coriolis_matrix, state)
            #
            # quad_motor_forces = dynamics.create_motor_forces_from_desired_torque_and_thrust(vehicle_torques, thrust)
            #
            # manipulator_control.calculate_joint_forces(inertia_matrix, pos_control_input, rotational_control_input,
            #                                            link_control_input, coriolis_matrix, g, state)

            e = inverse_kinematics._calculate_ee_error(desired_pos, state, desired_rot)
            pos_error = math.sqrt(e[0, 0]**2 + e[1, 0]**2 + e[2, 0]**2)
            rot_error = math.sqrt(e[3, 0]**2 + e[4, 0]**2 + e[5, 0]**2)

            visualization.plot_pos_error(pos_error, t)
            visualization.plot_rot_error(rot_error, t)

            pos_acceleration = controllable_vars[:3, :]
            rot_acceleration = numpy.array([[controllable_vars[3, 0]], [0], [0]])
            link_acceleration = controllable_vars[4:, :]
            new_state = system_emulator.apply_system_inputs(TIME_RESOLUTION_S, state, pos_acceleration,
                                                            rot_acceleration, link_acceleration)

            state = new_state
            print("State: " + str(state))
            visualization.plot_desired_point(desired_pos)

            ee_position = transformation_matrices.get_ee_position(state)
            visualization.plot_actual_point(ee_position)

            t += TIME_RESOLUTION_S
        except Exception as e:
            raise e

    visualization.show_plot()


if __name__ == '__main__':
    main()
