from dataclasses import dataclass
from urdfpy import xyz_rpy_to_matrix, matrix_to_xyz_rpy
from urdfModifiers.core import modifier
import math
import numpy as np
from urdfModifiers.core.jointModifier import JointModifier
from urdfModifiers.core.linkModifier import LinkModifier
from urdfModifiers.core.modification import Modification
from urdfModifiers.geometry import *
from urdfModifiers.geometry.geometry import Geometry, Side 

@dataclass
class FixedOffsetModifier():
    """Class to modify link that is Z-parallel with its previous and following joints, while keeping the offsets"""

    def __init__(self, link, robot):
        self.link = link
        self.link_modifier = LinkModifier(link, axis=Side.Z)
        parent_joint_list = [corresponding_joint for corresponding_joint in robot.joints if corresponding_joint.child == link.name]
        self.parent_joint = (parent_joint_list[0] if parent_joint_list else None)
        child_joint_list = [corresponding_joint for corresponding_joint in robot.joints if corresponding_joint.parent == link.name]
        self.child_joint = (child_joint_list[0] if child_joint_list else None)
        self.joint_modifier = JointModifier(self.child_joint, axis=Side.Z)
        self.check_if_z_parallel()

    @classmethod
    def from_name(cls, link_name, robot):
        """Creates an instance of FixedOffsetModifier by passing the robot object and link name"""
        return cls(FixedOffsetModifier.get_element_by_name(link_name, robot), robot)

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

    def check_if_z_parallel(self):
        """Validates that all Z axis of link and connected joints are parallel, otherwise raises an exception"""
        modifier_is_valid = True

        if (self.parent_joint is not None):
            link_origin = matrix_to_xyz_rpy(self.link.visuals[0].origin)
            modifier_is_valid = modifier_is_valid and link_origin[3] == 0 and link_origin[4] == 0

        if (self.child_joint is not None):
            child_joint_origin = matrix_to_xyz_rpy(self.child_joint.origin)
            modifier_is_valid = modifier_is_valid and child_joint_origin[3] == 0 and child_joint_origin[4] == 0

        if not modifier_is_valid:
            raise Exception("Cannot create FixedOffsetModifier for a setup that is not z-parallel")

    def get_significant_length(self):
        """Returns the significant length, for spheres it returns diameter instead of radius"""
        significant_length = self.link_modifier.get_significant_length()
        geometry_type, _ = self.get_geometry(self.link_modifier.get_visual())
        if geometry_type == Geometry.SPHERE:
            significant_length *= 2
        return significant_length

    def get_joint_origin(self, joint):
        """Returns the origin of a joint"""
        return (matrix_to_xyz_rpy(joint.origin) if joint else None)

    def get_link_origin(self, link):
        """Returns the origin of a joint"""
        return matrix_to_xyz_rpy(link.visuals[0].origin)

    def calculate_offsets(self):
        link_length = self.get_significant_length()
        link_origin = self.get_link_origin(self.link)
        link_origin_z = link_origin[2]
        child_joint_origin = self.get_joint_origin(self.child_joint)
        child_joint_origin_z = (child_joint_origin[2] if child_joint_origin is not None else 0)
        parent_joint_offset = (link_origin_z - link_length * math.copysign(1, child_joint_origin_z) / 2 if self.parent_joint else None)
        child_joint_offset = (link_origin_z + link_length * math.copysign(1, child_joint_origin_z) / 2 - child_joint_origin_z if self.child_joint else None)
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
                self.change_dimension_and_keep_offsets(modifications.dimension.value)
            else:
                self.change_dimension_and_keep_offsets(original_length * modifications.dimension.value)

    def change_dimension_and_keep_offsets(self, new_length):
        """Changes the dimension of the link while keeping the offset between it and both parent and child joints"""

        child_joint_origin = self.get_joint_origin(self.child_joint)
        child_joint_origin_z = (child_joint_origin[2] if child_joint_origin is not None else 0)

        parent_joint_offset, child_joint_offset = self.calculate_offsets()

        link_modification = Modification()

        if parent_joint_offset is not None:
            new_link_origin = parent_joint_offset + new_length * math.copysign(1, child_joint_origin_z) / 2
            link_modification.add_position(new_link_origin, absolute=True)
        else:
            # for the joint calculations, if there is no parent we position it as if it were in the center of the visual
            parent_joint_offset = -new_length / 2

        geometry_type, _ = self.get_geometry(self.link_modifier.get_visual())
        if geometry_type == Geometry.SPHERE:        
            link_modification.add_radius(new_length / 2, absolute=True)
        else:        
            link_modification.add_dimension(new_length, absolute=True)

        self.link_modifier.modify(link_modification)

        if (self.child_joint is not None):
            new_child_joint_origin = new_length * math.copysign(1, child_joint_origin_z) + parent_joint_offset - child_joint_offset
            joint_modification = Modification()

            joint_modification.add_position(new_child_joint_origin, absolute=True)

            self.joint_modifier.modify(joint_modification)
