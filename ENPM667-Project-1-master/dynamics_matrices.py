"""dynamics_matrices.py - contains the definitions for the intertia, coriolis, and gravitational matrices."""


import numpy
import jacobian


BODY_MASS = 2
GRAVITY_MS2 = -9.8


def get_inertial_matrix():
    # Inferred from Lippiello et al.
    M = numpy.eye(11)
    M[:3, :3] = BODY_MASS * numpy.eye(3)
    M[3:6, 3:6] = numpy.diag([1.24, 1.24, 2.48])
    # M[6:, 6:] # For now, assume the arm links to have no inertia.
    return M


def get_coriolis_matrix():
    # TODO - for now, we just use the identity matrix. This should be calculated from the model of the drone.
    return numpy.eye(11)


link_weights_grams = [80, 10, 6, 2, 1.5]
def get_gravity_matrix():
    g = numpy.ndarray((11, 1))
    g[2, 0] = GRAVITY_MS2

    # TODO - calculate the effect of gravity on each link in the arm

    return g