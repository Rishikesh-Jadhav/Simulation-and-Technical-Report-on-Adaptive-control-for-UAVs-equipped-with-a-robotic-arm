"""system_emulator.py - Emulates the system given the provided inputs and returns the new state after the
specified time step. Artificial noise is introduced when the new state is generated."""

import math
import system_state


def h(old_value, control_input):
    return old_value * 0.9 + control_input * 0.1


def apply_system_inputs(time_step, old_state, pos_control, rotation_control, link_control):
    """
    This function is a stand-in (mock) for a true simulation back-end.
    This naively creates a new state by applying the system derivative, along with some control influence,
    to create a new state. In the future, this can be replaced with a physics-based simulation such as Gazebo."""

    new_state = system_state.SystemState()
    new_state.body_x = old_state.body_x + old_state.vx * time_step
    new_state.body_y = old_state.body_y + old_state.vy * time_step
    new_state.body_z = old_state.body_z + old_state.vz * time_step

    new_state.yaw = old_state.yaw + old_state.rotational_velocity_yaw * time_step
    new_state.yaw = new_state.yaw % math.pi*2
    new_state.pitch = old_state.pitch + old_state.rotational_velocity_pitch * time_step
    new_state.pitch = new_state.pitch % math.pi*2
    new_state.roll = old_state.roll + old_state.rotational_velocity_roll * time_step
    new_state.roll = new_state.roll % math.pi*2

    for i in range(len(old_state.joint_positions)):
        new_pos = old_state.joint_positions[i] + old_state.joint_velocities[i] * time_step
        new_pos = new_pos % math.pi*2
        new_state.joint_positions[i] = new_pos

    new_state.vx = old_state.vx + h(old_state.ax, pos_control[0, 0]) * time_step
    new_state.vy = old_state.vy + h(old_state.ay, pos_control[1, 0]) * time_step
    new_state.vz = old_state.vz + h(old_state.az, pos_control[2, 0]) * time_step

    new_state.rotational_velocity_yaw = old_state.rotational_velocity_yaw + h(old_state.rot_accel_yaw, rotation_control[0, 0]) * time_step
    new_state.rotational_velocity_pitch = old_state.rotational_velocity_pitch + h(old_state.rot_accel_pitch, rotation_control[1, 0]) * time_step
    new_state.rotational_velocity_roll = old_state.rotational_velocity_roll + h(old_state.rot_accel_roll, rotation_control[2, 0]) * time_step

    for i in range(len(old_state.joint_velocities)):
        new_vel = old_state.joint_velocities[i] + h(old_state.joint_accelerations[i], link_control[i, 0]) * time_step
        new_state.joint_velocities[i] = new_vel

    new_state.ax = h(old_state.ax, pos_control[0, 0])
    new_state.ay = h(old_state.ay, pos_control[1, 0])
    new_state.az = h(old_state.az, pos_control[2, 0])

    new_state.rot_accel_yaw = h(old_state.rot_accel_yaw, rotation_control[0, 0])
    new_state.rot_accel_pitch = h(old_state.rot_accel_pitch, rotation_control[1, 0])
    new_state.rot_accel_roll = h(old_state.rot_accel_roll, rotation_control[2, 0])

    for i in range(len(old_state.joint_accelerations)):
        new_acc = h(old_state.joint_accelerations[i], link_control[i, 0])
        new_state.joint_accelerations[i] = new_acc

    return new_state
