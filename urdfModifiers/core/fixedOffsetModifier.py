from dataclasses import dataclass
from urdfpy import matrix_to_xyz_rpy
from math import isclose
import numpy as np
from urdfModifiers.core.jointModifier import JointModifier
from urdfModifiers.core.linkModifier import LinkModifier
from urdfModifiers.core.modification import Modification
from urdfModifiers.geometry import *
from urdfModifiers.geometry.geometry import Geometry, Side 

@dataclass
class Offset():
    """Class representing a three-dimensional offset between a joint and a link"""
    def __init__(self, joint=None, x=0, y=0, z=0):
        self.joint = joint
        self.x = x
        self.y = y
        self.z = z
    
    @classmethod
    def from_vector(cls, vector_array, joint=None):
        return cls(joint, x=vector_array[0], y=vector_array[1], z=vector_array[2])

    def to_vector(self):
        return np.array([self.x, self.y, self.z]).reshape((3,1))

    def __str__(self):
        joint_name = self.joint.name if self.joint else ''
        return f"Offset for Joint {joint_name} ({self.x=}, {self.y=}, {self.z=})"

    def __eq__(self, other):
        return (
            isclose(self.x, other.x) and
            isclose(self.y, other.y) and
            isclose(self.z, other.z)
        )

@dataclass
class FixedOffsetModifier():
    """
    Class to modify link that is Z-parallel with its previous and following joints, while keeping the offsets
    
        --- +-----------+ -
        ^   |           | ^
    e_o |   |     ^     | |
        |   |     |     | |
        --- |     o-->  | |-----------
            |   child   | |          ^
            |           | | v_l      |
            |           | |          |
            |     o---- | |----      |
            |   visual  | |   ^      | j_o
            |           | |   |      |
            |           | |   |      |
            |           | |   |      |
            |           | |   |      |
            |           | |   |      |
            |           | v   |      |
        --- +-----------+ -   | v_o  |
        ^         ^           |      |
    s_o |         |           |      |
        |         o-->        |      |
                parent

    Considering the above link, we define the following values:
    s_o - distance between the link frame (the parent origin referential) and the start of the link visual element      - parent_joint_offset
    e_o - distance between the child link frame (the child origin referential) and the end of the link visual element   - child_joint_offset

    j_o - distance between the axis connecting the link to its parent and the axis connecting the link to its child     - child_joint_origin
    v_l - length of the visual element (box or cylinder) of the link                                                    - link_length
    v_o - distance between the link frame and the center of the visual element                                          - link_visual_origin

    Using these definitions, we can relate them using the following formulas:

    s_o = v_o - v_l * sign(j_o) / 2
    e_o = v_o + v_l * sign(j_o) / 2 - j_o

    Using these formulas, and assuming we want to keep the values s_o and e_o constant, a modification of the link length would lead 
    to a known change of v_l, which we define as v_l'. Knowing v_l', s_o and e_o we compute the 2 remaining values, v_o' and j_o', therefore completely defining
    the origin of both the link visual and the child joint:

    v_o' = s_o + v_l' * sign(j_o) / 2
    j_o' = v_o' - e_o + v_l' * sign(j_o) / 2

    or, solving for v_o'

    j_o' = s_o + v_l' * sign(j_o) - e_o

    IMPORTANT NOTE:
    This fixed offset modification only works for z-parallel links, that is, when the parent joint, link and child joint XY planes are all parallel:

      /--^----/
     /   |   /     child XY plane
    /-------/

        /--^----/
       /   |   /   link XY plane
      /-------/

      /--^----/
     /   |   /     parent XY plane
    /-------/

    if the link you are changing is not z-parallel, you should instead use the API available in JointModifier and LinkModifier
            
    For more details regarding the calculation of these values, you can check the following link:
    https://github.com/icub-tech-iit/urdf-modifiers/issues/11
    
    """

    def __init__(self, link, robot, axis=Side.Z):
        self.link = link
        self.link_modifier = LinkModifier(link, axis=axis)
        parent_joint_list = [corresponding_joint for corresponding_joint in robot.joints if corresponding_joint.child == link.name]
        self.parent_joint = (parent_joint_list[0] if parent_joint_list else None)
        self.child_joint_list = [corresponding_joint for corresponding_joint in robot.joints if corresponding_joint.parent == link.name]
        self.joint_modifier_list = [JointModifier(item, axis = Side.Z) for item in self.child_joint_list]

    @classmethod
    def from_name(cls, link_name, robot, axis=Side.Z):
        """Creates an instance of FixedOffsetModifier by passing the robot object and link name"""
        return cls(FixedOffsetModifier.get_element_by_name(link_name, robot), robot, axis)

    @staticmethod
    def get_element_by_name(element_name, robot):
        """Explores the robot looking for the element whose name matches the first argument"""
        link_list = [corresponding_link for corresponding_link in robot.links if corresponding_link.name == element_name]
        joint_list = [corresponding_joint for corresponding_joint in robot.joints if corresponding_joint.name == element_name]
        if len(link_list) != 0:
            return link_list[0]
        elif len(joint_list) != 0:
            return joint_list[0]
        else:
            return None

    @staticmethod
    def get_geometry(geometry_holder):
        """Returns the geometry type and the corresponding geometry object for a given geometry holder (visual/collision)"""
        if geometry_holder is None:
            return [None, None]
        if (geometry_holder.geometry.box is not None):
            return [geometry.Geometry.BOX, geometry_holder.geometry.box]
        if (geometry_holder.geometry.cylinder is not None):
            return [geometry.Geometry.CYLINDER, geometry_holder.geometry.cylinder]
        if (geometry_holder.geometry.sphere is not None):
            return [geometry.Geometry.SPHERE, geometry_holder.geometry.sphere]

    def get_direction_vector(self):
        """Returns a numpy array corresponding to the relative direction of elongation of the modifier. For spheres and cylinders
        this vector points to the Z axis but in box geometries it could also point towards X or Y depending on the axis"""
        geometry_type, _ = self.get_geometry(self.link.visuals[0])
        if geometry_type == geometry.Geometry.SPHERE or geometry_type == geometry.Geometry.CYLINDER or self.link_modifier.axis == Side.Z:
            return np.array([[0],[0],[1]]) # if sphere, cylinder or axis is z return unit vector pointing to z
        
        if self.link_modifier.axis == Side.X:
            return np.array([[1],[0],[0]]) # unit vector pointing to x
        
        return np.array([[0],[1],[0]]) # unit vector pointing to y

    def get_significant_length(self):
        """Returns the significant length, for spheres it returns diameter instead of radius"""
        significant_length = self.link_modifier.get_significant_length()
        geometry_type, _ = self.get_geometry(self.link_modifier.get_visual())
        if geometry_type == Geometry.SPHERE:
            significant_length *= 2
        return significant_length

    def get_joint_origin(self, joint, transform = True):
        """Returns the origin of a joint w.r.t. to the link frame"""
        if not joint:
            return None
        
        return (matrix_to_xyz_rpy(joint.origin) if transform else joint.origin)

    def get_link_origin(self, link, transform = True):
        """Returns the origin of a first visual element w.r.t. to the link frame"""
        link_origin = matrix_to_xyz_rpy(link.visuals[0].origin) if transform else link.visuals[0].origin
        return link_origin

    def split_transformation_matrix(self, matrix):
        """Splits a transformation matrix into its rotation matrix and translation vector"""
        rotation_matrix = np.array(matrix[0:3,0:3]) 
        translation_vector = np.transpose(np.array(matrix[0:3,3])).reshape((3,1))
        return rotation_matrix, translation_vector


    def calculate_offsets(self):
        """Calculates the offsets between a link's extremes and its parent and children joints"""
        link_length = self.get_significant_length()

        link_origin_matrix = self.get_link_origin(self.link, transform=False)
        link_rotation_matrix, link_translation_vector = self.split_transformation_matrix(link_origin_matrix)

        unit_vector = self.get_direction_vector()

        parent_joint_offset = None
        child_joint_offset = []

        # Using formula: s_o = v_o - v_l / 2
        if self.parent_joint:
            offset_vector = link_translation_vector - link_length / 2 * np.dot(link_rotation_matrix, unit_vector)
            parent_joint_offset = Offset.from_vector(offset_vector.flatten(), joint=self.parent_joint)

        for item in self.child_joint_list:
            # Using formula: e_o = v_o + v_l * sign(j_o) / 2 - j_o
            joint_origin_matrix = self.get_joint_origin(item, transform=False)
            _, joint_translation_vector = self.split_transformation_matrix(joint_origin_matrix)
            offset_vector = link_translation_vector + link_length / 2 * np.dot(link_rotation_matrix, unit_vector) - joint_translation_vector
            child_joint_offset.append(Offset.from_vector(offset_vector.flatten(), joint=item))

        return parent_joint_offset, child_joint_offset

    def modify(self, modifications):
        """Performs the modifications in the link-joint setup"""

        trivial_modifications = Modification()
        if modifications.radius:
            trivial_modifications.add_radius(modifications.radius.value, modifications.radius.absolute)
        if modifications.density:
            trivial_modifications.add_density(modifications.density.value, modifications.density.absolute)
        if modifications.mass:
            trivial_modifications.add_mass(modifications.mass.value, modifications.mass.absolute)
        
        self.link_modifier.modify(trivial_modifications)

        if modifications.dimension:
            original_length = self.get_significant_length()
            if modifications.dimension.absolute:
                self.change_dimension_and_keep_offsets(modifications.dimension.value, modifications.offset_mask)
            else:
                self.change_dimension_and_keep_offsets(original_length * modifications.dimension.value, modifications.offset_mask)

    def change_dimension_and_keep_offsets(self, new_length, offset_mask):
        """Changes the dimension of the link while keeping the offset between it and both parent and child joints"""
        parent_joint_offset, child_joint_offset = self.calculate_offsets()
        unit_vector = self.get_direction_vector()

        # Change dimension
        link_modification = Modification()
        geometry_type, _ = self.get_geometry(self.link_modifier.get_visual())
        if geometry_type == Geometry.SPHERE:        
            link_modification.add_radius(new_length / 2, absolute=True)
        else:        
            link_modification.add_dimension(new_length, absolute=True)

        self.link_modifier.modify(link_modification)

        # Adjust link's origin to keep parent offset
        if parent_joint_offset is not None:
            # Using formula: v_o' = s_o + v_l' * sign(j_o) / 2 
            link_origin_matrix = self.get_link_origin(self.link, transform=False)
            link_rotation_matrix, _ = self.split_transformation_matrix(link_origin_matrix)

            new_parent_origin_position = parent_joint_offset.to_vector() + new_length / 2 * np.dot(link_rotation_matrix, unit_vector)
            
            self.modify_origin_three_dimensions(self.link_modifier, new_parent_origin_position, offset_mask)
        else:
            # for the joint calculations, if there is no parent we position it as if it were in the center of the visual
            # s_o = v_o - v_l * sign(j_o) / 2   with  v_o = 0
            parent_joint_offset = Offset(z=-new_length / 2)

        
        # Adjust child links' origins to keep children offsets
        link_origin_matrix = self.get_link_origin(self.link, transform=False) # Needs to be recalculated since the origin may have shifted
        link_rotation_matrix, link_translation_vector = self.split_transformation_matrix(link_origin_matrix)
        for item in child_joint_offset:
            # j_o' = s_o + v_l' * sign(j_o) - e_o
            new_child_origin_position = - item.to_vector() + link_translation_vector + new_length / 2 * np.dot(link_rotation_matrix, unit_vector)
        
            corresponding_modifier = [joint_modifier for joint_modifier in self.joint_modifier_list if joint_modifier.element == item.joint][0]
            self.modify_origin_three_dimensions(corresponding_modifier, new_child_origin_position, offset_mask)

    def modify_origin_three_dimensions(self, modifier, new_position, offset_mask=[1,1,1]):
        """Performs 3 position modifications to place the origin in a new X, Y and Z"""
        original_modifier_axis = modifier.axis
        modification = Modification()

        flattened_position_array = new_position.flatten()  

        if offset_mask[0]:
            modifier.axis = Side.X
            modification.add_position(flattened_position_array[0], absolute=True)
            modifier.modify(modification)

        if offset_mask[1]:
            modifier.axis = Side.Y
            modification.add_position(flattened_position_array[1], absolute=True)
            modifier.modify(modification)

        if offset_mask[2]:
            modifier.axis = Side.Z
            modification.add_position(flattened_position_array[2], absolute=True)
            modifier.modify(modification)

        modifier.axis = original_modifier_axis
