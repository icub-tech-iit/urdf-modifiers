from enum import Enum, EnumMeta, auto

class ContainsBasedOnKeyMeta(EnumMeta):
    """Metaclass for enums that modifies the __contains__ method to check for keys instead of values."""
    def __contains__(cls, item):
        try:
            cls[item]
        except KeyError:
            return False
        return True

class Geometry(Enum):
    """The different types of geometries that constitute the URDF"""
    BOX = auto()
    CYLINDER = auto()
    SPHERE = auto()

class Side(Enum):
    """The possible sides of a box geometry"""
    WIDTH = auto()
    HEIGHT = auto()
    DEPTH = auto()

class Limb(Enum, metaclass=ContainsBasedOnKeyMeta):
    """The possible limbs of the robot"""
    NONE = auto()
    RIGHT_ARM = auto()
    LEFT_ARM = auto()
    RIGHT_LEG = auto()
    LEFT_LEG = auto()
    ARMS = auto()
    LEGS = auto()
    TORSO = auto()
    ALL = auto()

class RobotElement(Enum):
    """Types of elements in the urdf"""
    LINK = auto()
    JOINT = auto()

class Modification():
    """Available modifications type"""
    DENSITY = "density"
    DIMENSION = "dimension"
    RADIUS = "radius"
    MASS = "mass"
    ABSOLUTE = True
    MULTIPLIER = False 
    SCALE= "_scale"
