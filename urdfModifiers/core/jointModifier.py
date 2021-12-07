from urdfModifiers.core import modifier
from urdfModifiers.core.linkModifier import LinkModifier
from urdfModifiers.geometry.geometry import *
from urdfpy import xyz_rpy_to_matrix, matrix_to_xyz_rpy 

class JointModifier(modifier.Modifier):
    """Class to modify joints in a URDF"""
    def __init__(self, joint, origin_modifier, parent, take_half_length = False, flip_direction = True):
        super().__init__(joint, origin_modifier, RobotElement.JOINT)
        self.take_half_length = take_half_length
        self.flip_direction = flip_direction
        self.parent = parent
        

    @classmethod
    def from_name(cls, joint_name, robot, origin_modifier, take_half_length = False, flip_direction = True):
        """Creates an instance of LinkModifier by passing the robot object and link name"""
        joint = JointModifier.get_element_by_name(joint_name, robot)
        parent = LinkModifier.get_element_by_name(joint.parent, robot)
        return cls(joint, origin_modifier, parent, take_half_length, flip_direction)

    @staticmethod
    def get_element_by_name(joint_name, robot):
        """Explores the robot looking for the joint whose name matches the first argument"""
        joint_list = [corresponding_joint for corresponding_joint in robot.joints if corresponding_joint.name == joint_name]
        if len(joint_list) != 0:
            return joint_list[0]
        else:
            return None

    def modify(self, modifications = None):
        """Performs the dimension and density modifications to the current link"""
        significant_length = self.get_parent_significant_length()
        self.modify_origin(significant_length)

    def get_parent_significant_length(self):
        """Gets the significant length of the parent link that defines the new position of the joint"""
        parent_visual_obj = LinkModifier.get_visual_static(self.parent)
        parent_geometry_type, parent_visual_data = LinkModifier.get_geometry(parent_visual_obj)
        if (parent_geometry_type == Geometry.CYLINDER):
            return parent_visual_data.length
        elif (parent_geometry_type == Geometry.BOX):
            return parent_visual_data.size.max()
        else:
            return 0

    def modify_origin(self, length):
        """Modifies the position of the origin by a given amount"""
        xyz_rpy = matrix_to_xyz_rpy(self.element.origin)
        xyz_rpy[2] = length / (2 if self.take_half_length else 1)
        if self.flip_direction:
            xyz_rpy[2] *= -1
        xyz_rpy[2] += self.origin_modifier
        self.element.origin = xyz_rpy_to_matrix(xyz_rpy)