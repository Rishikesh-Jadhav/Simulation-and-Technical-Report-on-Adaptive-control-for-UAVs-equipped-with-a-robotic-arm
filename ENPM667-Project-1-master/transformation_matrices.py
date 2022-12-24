"""transformation_matrices.py - contains functions to express data in the world reference frame."""

import sympy
import jacobian
import numpy


yaw, pitch, roll, robot_x, robot_y, robot_z = sympy.symbols("yaw, pitch, roll, robot_x, robot_y, robot_z")

ROTATION_MATRIX_BASE_TO_WORLD = sympy.Matrix([[sympy.cos(yaw) * sympy.cos(pitch),
                                               sympy.cos(yaw) * sympy.sin(pitch) * sympy.sin(roll) - sympy.sin(yaw) * sympy.cos(roll),
                                               sympy.cos(yaw) * sympy.sin(pitch) * sympy.cos(roll) + sympy.sin(yaw) * sympy.sin(roll)],
                                              [sympy.sin(yaw) * sympy.cos(pitch),
                                               sympy.sin(yaw) * sympy.sin(pitch) * sympy.sin(roll) + sympy.cos(yaw) * sympy.cos(roll),
                                               sympy.sin(yaw) * sympy.sin(pitch) * sympy.cos(roll) - sympy.cos(yaw) * sympy.sin(roll)],
                                              [-1 * sympy.sin(pitch),
                                               sympy.cos(pitch) * sympy.sin(roll),
                                               sympy.cos(pitch) * sympy.cos(roll)]])

TRANSFORM_MAT_EULER_TO_ANG_VEL = sympy.Matrix([[0, -sympy.sin(yaw), sympy.cos(yaw)*sympy.cos(pitch)],
                                               [0, sympy.cos(yaw), sympy.sin(yaw)*sympy.cos(pitch)],
                                               [1, 0, -sympy.sin(pitch)]])


def get_instantaneous_rotation_matrix(state):
    rotate_base_to_world_matrix = ROTATION_MATRIX_BASE_TO_WORLD.subs([(yaw, state.yaw),
                                                                      (pitch, state.pitch),
                                                                      (roll, state.roll)])
    return rotate_base_to_world_matrix


def get_position_in_world_from_base_frame(state, point):
    rotate_base_to_world_matrix = ROTATION_MATRIX_BASE_TO_WORLD.subs([(yaw, state.yaw),
                                                                       (pitch, state.pitch),
                                                                       (roll, state.roll)])

    translation_base_to_world = sympy.Matrix([[state.body_x], [state.body_y], [state.body_z]])

    point = numpy.array([[point[0, 0]], [point[1, 0]], [point[2, 0]]])
    output_coord = translation_base_to_world + rotate_base_to_world_matrix * point
    return output_coord


def get_linear_velocity_in_world_from_base(state, point, point_velocity):

    rotate_base_to_world_matrix = ROTATION_MATRIX_BASE_TO_WORLD.subs([(yaw, state.yaw),
                                                                       (pitch, state.pitch),
                                                                       (roll, state.roll)])

    base_velocity = sympy.Matrix([state.vx], [state.vy], [state.vz])
    base_rotational_velocity = sympy.Matrix([state.vyaw], [state.vpitch], [state.vroll])

    point_rotated_to_world = rotate_base_to_world_matrix * point
    output_velocity = base_velocity - point_rotated_to_world.cross(base_rotational_velocity) + rotate_base_to_world_matrix * point_velocity

    return output_velocity


def get_angular_velocity_in_world_from_base(state, point_rotational_vel):
    rotate_base_to_world_matrix = ROTATION_MATRIX_BASE_TO_WORLD.subs([(yaw, state.yaw),
                                                                      (pitch, state.pitch),
                                                                      (roll, state.roll)])

    base_rotational_velocity = sympy.Matrix([state.vyaw], [state.vpitch], [state.vroll])
    output_rotational_vel = base_rotational_velocity + rotate_base_to_world_matrix * point_rotational_vel

    return output_rotational_vel


def get_angular_velocity_transformation_matrix(state):
    transform_euler_angle_deriv_to_angular_velocity = TRANSFORM_MAT_EULER_TO_ANG_VEL.subs([(yaw, state.yaw),
                                                                                           (pitch, state.pitch),
                                                                                           (roll, state.roll)])
    return transform_euler_angle_deriv_to_angular_velocity


def get_ee_position(state):
    ee_to_base_transform_matrix = jacobian.T0_n[-1]
    ee_position_in_base_frame = ee_to_base_transform_matrix * numpy.array([[0], [0], [0], [1]])
    ee_position_in_world_frame = get_position_in_world_from_base_frame(state, ee_position_in_base_frame)
    arm_state = state.joint_positions
    ee_position_in_world_frame = ee_position_in_world_frame.subs([(jacobian.q1, arm_state[0]),
                                                                  (jacobian.q2, arm_state[1]),
                                                                  (jacobian.q3, arm_state[2]),
                                                                  (jacobian.q4, arm_state[3]),
                                                                  (jacobian.q5, arm_state[4])])
    return ee_position_in_world_frame
