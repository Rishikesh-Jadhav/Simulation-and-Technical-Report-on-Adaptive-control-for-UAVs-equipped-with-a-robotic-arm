"""jacobian.py - contains methods pertaining to the Jacobian and its use."""

import numpy
import sympy

import transformation_matrices


q1, q2, q3, q4, q5, t = sympy.symbols("q1, q2, q3, q4, q5, t")
GENERIC_JACOBIAN = sympy.Matrix((6, 5))

# D-H parameters for manipulator arm, from Arleo et al: Control of Quadrotor Aerial Vehicles Equipped with a Robotic Arm
THETAS = [q1, q2, q3, q4, q5]
DS = [0, 0, 0, 0, .18]
ALPHAS = [-numpy.pi/2, numpy.pi/2, 0, -numpy.pi/2, 0]
AS = [0, .150, .80, 0, 0]


def _generate_skew_matrix(a):
    skew_matrix = numpy.array([[0, -a[2, 0], a[1, 0]],
                                [a[2, 0], 0, -a[0, 0]],
                                [-a[1, 0], a[0, 0], 0]])
    return skew_matrix


def get_effector_velocity_in_base_frame(arm_state, joint_velocities):
    instantaneous_jacobian = GENERIC_JACOBIAN.subs([(q1, arm_state.q1),
                                                    (q2, arm_state.q2),
                                                    (q3, arm_state.q3),
                                                    (q4, arm_state.q4),
                                                    (q5, arm_state.q5)])
    effector_velocity = instantaneous_jacobian * joint_velocities
    return effector_velocity


def get_effector_velocity_in_world_frame(state, end_effector_position, arm_state, joint_velocities):
    j_b = numpy.eye(6)
    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)
    skew_matrix = _generate_skew_matrix(rotation_matrix * end_effector_position)
    j_b[0:3, 3:6] = -1 * skew_matrix

    j_eb = numpy.zeros(6)
    j_eb[0:3, 0:3] = rotation_matrix
    j_eb[3:6, 3:6] = rotation_matrix
    instantaneous_jacobian = GENERIC_JACOBIAN.subs([(q1, arm_state.q1),
                                                    (q2, arm_state.q2),
                                                    (q3, arm_state.q3),
                                                    (q4, arm_state.q4),
                                                    (q5, arm_state.q5)])
    j_eb = j_eb * instantaneous_jacobian

    base_frame_velocity = sympy.Matrix([[state.vx], [state.vy], [state.vz], [state.vyaw], [state.vpitch], [state.vroll]])
    effector_velocity_in_world_frame = j_b * base_frame_velocity + j_eb * joint_velocities
    return effector_velocity_in_world_frame


def get_end_effector_velocity(state, end_effector_position, arm_state):
    j_b = numpy.eye(6)
    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)
    skew_matrix = _generate_skew_matrix(rotation_matrix * end_effector_position)
    j_b[0:3, 3:6] = -1 * skew_matrix

    t_a = numpy.eye(6)
    t_of_phi = sympy.Matrix([[0, -sympy.sin(state.yaw), sympy.cos(state.yaw) * sympy.cos(state.pitch)],
                             [0, sympy.cos(state.yaw), sympy.sin(state.yaw) * sympy.cos(state.pitch)],
                             [1, 0, -1 * sympy.sin(state.pitch)]])
    t_a[3:6, 3:6] = t_of_phi

    j_b_t_a = j_b * t_a
    j_sigma = j_b_t_a[:, -2:]

    j_nu = j_b_t_a[:, :4]

    j_eb = numpy.zeros(6)
    j_eb[0:3, 0:3] = rotation_matrix
    j_eb[3:6, 3:6] = rotation_matrix
    instantaneous_jacobian = GENERIC_JACOBIAN.subs([(q1, arm_state.q1),
                                                    (q2, arm_state.q2),
                                                    (q3, arm_state.q3),
                                                    (q4, arm_state.q4),
                                                    (q5, arm_state.q5)])
    j_eb = j_eb * instantaneous_jacobian

    j_epsilon = numpy.zeros((6, 10))
    j_epsilon[:, -6:] = j_eb
    j_epsilon[:, :4] = j_nu

    controllable_variable_velocities = numpy.array([[state.vx], [state.vy], [state.vz], [state.vyaw]])
    uncontrollable_variable_velocities = numpy.array([[state.vpitch], [state.vroll]])
    end_effector_velocity = j_epsilon * controllable_variable_velocities + j_sigma * uncontrollable_variable_velocities

    return end_effector_velocity


# Generates a list of transformation matrices from the base_link to each frame N of the manipulator arm.
# Adapted from a previous project for Robotic Control
def transformation_matrix(a, alpha, d, theta):
    one_step_transforms = []  # Will store matrix from T0_n frames
    A = numpy.identity(4)
    dh_table = numpy.array([[a[0], alpha[0], d[0], theta[0]],
                         [a[1], alpha[1], d[1], theta[1]],
                         [a[2], alpha[2], d[2], theta[2]],
                         [a[3], alpha[3], d[3], theta[3]],
                         [a[4], alpha[4], d[4], theta[4]]])

    for i in range(0, len(dh_table)):
        T = sympy.Matrix([[sympy.cos(dh_table[i, 3]), -sympy.sin(dh_table[i, 3]) * sympy.cos(dh_table[i, 1]),
                           sympy.sin(dh_table[i, 3]) * sympy.sin(dh_table[i, 1]),
                           dh_table[i, 0] * sympy.cos(dh_table[i, 3])],
                          [sympy.sin(dh_table[i, 3]), sympy.cos(dh_table[i, 3]) * sympy.cos(dh_table[i, 1]), -sympy.cos(
                              dh_table[i, 3]) * sympy.sin(dh_table[i, 1]), dh_table[i, 0] * sympy.sin(dh_table[i, 3])],
                          [0, sympy.sin(dh_table[i, 1]), sympy.cos(
                              dh_table[i, 1]), dh_table[i, 2]],
                          [0, 0, 0, 1]])

        A = A @ T

        one_step_transforms.append(T)
        # with np.printoptions(precision=2, suppress=True):
        #    print(A)

    A = numpy.identity(4)
    matrix_list = []
    A = A @ one_step_transforms[0]
    matrix_list.append(A)  # Appending transform to frame 1
    A = A @ one_step_transforms[1]
    matrix_list.append(A)  # Appending transform to frame 2
    A = A @ one_step_transforms[2]
    matrix_list.append(A)  # Appending transform to frame 3
    A = A @ one_step_transforms[3]
    matrix_list.append(A)
    A = A @ one_step_transforms[4]
    matrix_list.append(A)

    return matrix_list


def generate_generic_jacobian(ts):
    global GENERIC_JACOBIAN

    j1 = sympy.zeros(6, 1)
    j1[0:3, 0] = sympy.Matrix([[0], [0], [1]]).cross(ts[-1][0:3, 3] - sympy.Matrix([[0], [0], [0]]))
    j1[3:6, 0] = sympy.Matrix([[0], [0], [1]])

    j2 = sympy.zeros(6, 1)
    j2[0:3, 0] = ts[0][0:3, 2].cross(ts[-1][0:3, 3] - ts[0][0:3, 3])
    j2[3:6, 0] = ts[0][0:3, 2]

    j3 = sympy.zeros(6, 1)
    j3[0:3, 0] = ts[1][0:3, 2].cross(ts[-1][0:3, 3] - ts[1][0:3, 3])
    j3[3:6, 0] = ts[1][0:3, 2]

    j4 = sympy.zeros(6, 1)
    j4[0:3, 0] = ts[2][0:3, 2].cross(ts[-1][0:3, 3] - ts[2][0:3, 3])
    j4[3:6, 0] = ts[2][0:3, 2]

    j5 = sympy.zeros(6, 1)
    j5[0:3, 0] = ts[3][0:3, 2].cross(ts[-1][0:3, 3] - ts[3][0:3, 3])
    j5[3:6, 0] = ts[3][0:3, 2]

    GENERIC_JACOBIAN = sympy.zeros(6, 5)
    GENERIC_JACOBIAN[:, 0] = j1
    GENERIC_JACOBIAN[:, 1] = j2
    GENERIC_JACOBIAN[:, 2] = j3
    GENERIC_JACOBIAN[:, 3] = j4
    GENERIC_JACOBIAN[:, 4] = j5


def get_current_jacobian(joint_positions):
    instantaneous_jacobian = GENERIC_JACOBIAN.subs([(q1, joint_positions[0]),
                                                    (q2, joint_positions[1]),
                                                    (q3, joint_positions[2]),
                                                    (q4, joint_positions[3]),
                                                    (q5, joint_positions[4])])
    return instantaneous_jacobian


def get_jacobian_of_controllable_variables(arm_state, state, end_effector_position):

    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)
    rot_matrix = numpy.zeros((6, 6))
    rot_matrix[0:3, 0:3] = rotation_matrix
    rot_matrix[3:6, 3:6] = rotation_matrix
    instantaneous_jacobian = GENERIC_JACOBIAN.subs([(q1, arm_state[0]),
                                                    (q2, arm_state[1]),
                                                    (q3, arm_state[2]),
                                                    (q4, arm_state[3]),
                                                    (q5, arm_state[4])])
    j_eb = rot_matrix * instantaneous_jacobian

    j_b = numpy.eye(6)
    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)
    skew_matrix = _generate_skew_matrix(rotation_matrix * end_effector_position)
    j_b[0:3, 3:6] = -1 * skew_matrix
    t_a = numpy.eye(6)
    t_a[3:6, 3:6] = transformation_matrices.get_angular_velocity_transformation_matrix(state)
    j_n = (j_b * t_a)[:, 0:4]

    controllable_jacobian = numpy.ndarray((6, 9))
    controllable_jacobian[:, 0:4] = j_n
    controllable_jacobian[:, 4:] = j_eb

    return controllable_jacobian


def get_deriv_of_controlled_jacobian(arm_state, arm_vels, state, end_effector_position):
    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)
    j_eb = numpy.zeros((6, 6))
    j_eb[0:3, 0:3] = rotation_matrix
    j_eb[3:6, 3:6] = rotation_matrix
    jacobian_in_terms_of_t = GENERIC_JACOBIAN.subs([(q1, arm_state[0] + arm_vels[0] * t),
                                                    (q2, arm_state[1] + arm_vels[1] * t),
                                                    (q3, arm_state[2] + arm_vels[2] * t),
                                                    (q4, arm_state[3] + arm_vels[3] * t),
                                                    (q5, arm_state[4] + arm_vels[4] * t)])
    deriv_of_jacobian = sympy.diff(jacobian_in_terms_of_t, t)
    instantaneous_jacobian = deriv_of_jacobian.subs([(t, 0)])
    j_eb = j_eb * instantaneous_jacobian

    # TODO - how do we properly differentiate j_n?
    j_b = numpy.eye(6)
    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)
    skew_matrix = _generate_skew_matrix(rotation_matrix * end_effector_position)
    j_b[0:3, 3:6] = -1 * skew_matrix
    t_a = numpy.eye(6)
    t_a[3:6, 3:6] = transformation_matrices.get_angular_velocity_transformation_matrix(state)
    j_n = (j_b * t_a)[:, 0:4]

    controllable_jacobian = numpy.ndarray((6, 9))
    controllable_jacobian[:, 0:4] = j_n
    controllable_jacobian[:, 4:] = j_eb

    return controllable_jacobian


def get_jacobian_of_uncontrolled_variables(state, end_effector_position):
    rotation_matrix = transformation_matrices.get_instantaneous_rotation_matrix(state)

    j_b = numpy.eye(6)
    skew_matrix = _generate_skew_matrix(rotation_matrix * end_effector_position)
    j_b[0:3, 3:6] = -1 * skew_matrix
    t_a = numpy.eye(6)
    t_a[3:6, 3:6] = transformation_matrices.get_angular_velocity_transformation_matrix(state)
    j_b_t_a = j_b * t_a

    uncontrollable_jacobian = numpy.ndarray((6, 2))
    uncontrollable_jacobian[:, :] = j_b_t_a[:, -2:]

    return uncontrollable_jacobian


def get_derivative_of_uncontrolled_jacobian(state, end_effector_position):
    pass


T0_n = transformation_matrix(AS, ALPHAS, DS, THETAS)
generate_generic_jacobian(T0_n)


if __name__ == '__main__':
    # TODO - write test cases to verify the Jacobian
    pass