"""gain_constants.py - contains definitions for our gain constants used elsewhere in the paper."""

import numpy

K_P_P = 12 * numpy.eye(3)
K_P_V = 5 * numpy.eye(3)
K_PSI_P = 8
K_PSI_V = 3
K_PHI_P = 2
K_PHI_V = 1
BETA = 4

K_Q_P = 140 * numpy.eye(5)
K_Q_V = 20 * numpy.eye(5)
K_THETA_P = 2
K_THETA_V = 1
K_P = 7.5 * numpy.eye(6)
K_V = 0.6 * numpy.eye(6)