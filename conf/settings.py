from __future__ import print_function


class BlockSettings(object):
    # Types of block
    CUBE = 1
    CYLINDER = 2
    PRISM = 3
    PUCK = 4
    HOLE = 5
    RECT = 6
    UNKNOWN = 7

    TYPE_TO_STR = {
        CUBE: "cube",
        CYLINDER: "cylinder",
        PRISM: "prism",
        PUCK: "puck",
        HOLE: "hole",
        RECT: "Rect",
        UNKNOWN: "unknown",
        "???": "?!?"
    }

    BASIC_LENGTH = 35.0
    PUCK_DIAMETER = 28.5
    HOLE_DIAMETER = 50.0

    nBlockElements = 12  # This will also get update by block.py


class RobotSettings(object):
    LINK_1_LENGTH = 30.83443334*10  # 299.8629
    LINK_2_LENGTH = 25.87478254*10  # 250.9554
    BASE_Y_OFFSET = 17.02850786*10  # 157.848
    BASE_X_OFFSET = 0.020449064*10  # -1.1991


class WorkspaceSettings(object):
    WIDTH = 600.0
    HEIGHT = 300.0
