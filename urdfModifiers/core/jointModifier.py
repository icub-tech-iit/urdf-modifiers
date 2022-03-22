from urdfModifiers.core import modifier
from urdfModifiers.geometry.geometry import *
from urdfpy import xyz_rpy_to_matrix, matrix_to_xyz_rpy 

class JointModifier(modifier.Modifier):
    """Class to modify joints in a URDF"""
    def __init__(self, joint, axis = None):
        super().__init__(joint, RobotElement.JOINT)
        self.axis = axis
        

    @classmethod
    def from_name(cls, joint_name, robot, axis = None):
        """Creates an instance of LinkModifier by passing the robot object and link name"""
        joint = JointModifier.get_element_by_name(joint_name, robot)
        return cls(joint, axis)

    @staticmethod
    def get_element_by_name(joint_name, robot):
        """Explores the robot looking for the joint whose name matches the first argument"""
        joint_list = [corresponding_joint for corresponding_joint in robot.joints if corresponding_joint.name == joint_name]
        if len(joint_list) != 0:
            return joint_list[0]
        else:
            return None

    def modify(self, modifications):
        """Performs the position modifications to the current joint"""
        if modifications.position:
            if self.axis is None:
                raise Exception('Axis not specified for joint')
            xyz_rpy = matrix_to_xyz_rpy(self.element.origin)
            original_x = xyz_rpy[0]
            original_y = xyz_rpy[1]
            original_z = xyz_rpy[2]
            if modifications.position.absolute:
                if (self.axis == Side.X):
                    xyz_rpy[0] = modifications.position.value
                elif (self.axis == Side.Y):
                    xyz_rpy[1] = modifications.position.value
                elif (self.axis == Side.Z):
                    xyz_rpy[2] = modifications.position.value
            else:
                if (self.axis == Side.X):
                    xyz_rpy[0] = original_x * modifications.position.value
                elif (self.axis == Side.Y):
                    xyz_rpy[1] = original_y * modifications.position.value
                elif (self.axis == Side.Z):
                    xyz_rpy[2] = original_z * modifications.position.value
            self.element.origin = xyz_rpy_to_matrix(xyz_rpy) 
            
        if modifications.joint_type: 
            self.element.joint_type = modifications.joint_type
                
