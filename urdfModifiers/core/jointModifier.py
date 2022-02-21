from urdfModifiers.core import modifier
from urdfModifiers.geometry.geometry import *
from urdfpy import xyz_rpy_to_matrix, matrix_to_xyz_rpy 

class JointModifier(modifier.Modifier):
    """Class to modify joints in a URDF"""
    def __init__(self, joint, dimension = None):
        super().__init__(joint, RobotElement.JOINT)
        self.dimension = dimension
        

    @classmethod
    def from_name(cls, joint_name, robot, dimension = None):
        """Creates an instance of LinkModifier by passing the robot object and link name"""
        joint = JointModifier.get_element_by_name(joint_name, robot)
        return cls(joint, dimension)

    @staticmethod
    def get_element_by_name(joint_name, robot):
        """Explores the robot looking for the joint whose name matches the first argument"""
        joint_list = [corresponding_joint for corresponding_joint in robot.joints if corresponding_joint.name == joint_name]
        if len(joint_list) != 0:
            return joint_list[0]
        else:
            return None

    def modify(self, modifications):
        """Performs the dimension and density modifications to the current link"""
        if modifications.dimension:
            if self.dimension is None:
                raise Exception('Dimension not specified for joint')
            xyz_rpy = matrix_to_xyz_rpy(self.element.origin)
            original_x = xyz_rpy[0]
            original_y = xyz_rpy[1]
            original_z = xyz_rpy[2]
            if modifications.dimension.absolute:
                if (self.dimension == Side.X):
                    xyz_rpy[0] = modifications.dimension.value
                elif (self.dimension == Side.Y):
                    xyz_rpy[1] = modifications.dimension.value
                elif (self.dimension == Side.Z):
                    xyz_rpy[2] = modifications.dimension.value
            else:
                if (self.dimension == Side.X):
                    xyz_rpy[0] = original_x * modifications.dimension.value
                elif (self.dimension == Side.Y):
                    xyz_rpy[1] = original_y * modifications.dimension.value
                elif (self.dimension == Side.Z):
                    xyz_rpy[2] = original_z * modifications.dimension.value
            self.element.origin = xyz_rpy_to_matrix(xyz_rpy) 
                

    # def get_parent_significant_length(self):
    #     """Gets the significant length of the parent link that defines the new position of the joint"""
    #     parent_visual_obj = LinkModifier.get_visual_static(self.parent)
    #     parent_geometry_type, parent_visual_data = LinkModifier.get_geometry(parent_visual_obj)
    #     if (parent_geometry_type == Geometry.CYLINDER):
    #         return parent_visual_data.length
    #     elif (parent_geometry_type == Geometry.BOX):
    #         return parent_visual_data.size.max()
    #     else:
    #         return 0

    def modify_origin(self, length):
        """Modifies the position of the origin by a given amount"""
        xyz_rpy = matrix_to_xyz_rpy(self.element.origin)
        xyz_rpy[2] = length / (2 if self.take_half_length else 1)
        if self.flip_direction:
            xyz_rpy[2] *= -1
        xyz_rpy[2] += self.origin_modifier
        self.element.origin = xyz_rpy_to_matrix(xyz_rpy)