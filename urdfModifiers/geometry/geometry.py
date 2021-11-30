from enum import Enum, EnumMeta

class LimbMeta(EnumMeta):
    def __contains__(cls, item):
        try:
            cls[item]
        except KeyError:
            return False
        return True

class Geometry(Enum):
    """The different types of geometries that constitute the URDF"""
    BOX = 1
    CYLINDER = 2
    SPHERE = 3

class Side(Enum):
    """The possible sides of a box geometry"""
    WIDTH = 1
    HEIGHT = 2
    DEPTH = 3

class Limb(Enum, metaclass=LimbMeta):
    """The possible limbs of the robot"""
    NONE = 0
    RIGHT_ARM = 1
    LEFT_ARM = 2
    RIGHT_LEG = 3
    LEFT_LEG = 4
    ARMS = 5
    LEGS = 6
    TORSO = 7
    ALL = 8

class RobotElement(Enum):
    """Types of elements in the urdf"""
    LINK = 1
    JOINT = 2