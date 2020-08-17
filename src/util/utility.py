"""
Owner: Andrea Coletta
Version v1.0
Release code for TMC : 10.1109/TMC.2020.2994529 and its ICDCS conference version : 10.1109/ICDCS.2019.00209

Please cite these works in case of use.


File content:
This file contains all the main utility functions needed to run the AC-GaP and AC-TCP code.
"""

import math


def euclidean_distance(point1, point2):
    return math.sqrt(math.pow(point2[0] - point1[0], 2)
                     + math.pow(point2[1] - point1[1], 2))
