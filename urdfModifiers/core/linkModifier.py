from dataclasses import dataclass
from urdfpy import xyz_rpy_to_matrix, matrix_to_xyz_rpy
from urdfModifiers.core import modifier
import math
import numpy as np
from urdfModifiers.geometry import * 

@dataclass
class LinkModifier(modifier.Modifier):
    """Class to modify links in a URDF"""
    def __init__(self, link, origin_modifier, dimension = None, flip_direction = True, calculate_origin_from_dimensions = True):
        super().__init__(link, origin_modifier, geometry.RobotElement.LINK)
        self.dimension = dimension
        self.flip_direction = flip_direction
        self.calculate_origin_from_dimensions = calculate_origin_from_dimensions

    @classmethod
    def from_name(cls, link_name, robot, origin_modifier, dimension = None, flip_direction = True, calculate_origin_from_dimensions = True):
        """Creates an instance of LinkModifier by passing the robot object and link name"""
        return cls(LinkModifier.get_element_by_name(link_name, robot), origin_modifier, dimension, flip_direction, calculate_origin_from_dimensions)

    @staticmethod
    def get_element_by_name(link_name, robot):
        """Explores the robot looking for the link whose name matches the first argument"""
        link_list = [corresponding_link for corresponding_link in robot.links if corresponding_link.name == link_name]
        if len(link_list) != 0:
            return link_list[0]
        else:
            return None

    def modify(self, modifications):
        """Performs the dimension and density modifications to the current link"""
        original_density = self.calculate_density()
        original_radius = self.get_radius()
        original_length = self.get_significant_length()
        original_mass = self.get_mass()
        if modifications.radius:
            if modifications.radius.absolute:
                self.set_radius(modifications.radius.value)
            else:
                if original_radius is not None:
                    self.set_radius(original_radius * modifications.radius.value)
        if modifications.dimension:
            if modifications.dimension.absolute:
                self.set_length(modifications.dimension.value)
            else:
                if original_length is not None:
                    self.set_length(original_length * modifications.dimension.value)
        if modifications.density:
            if modifications.density.absolute:
                self.set_density(modifications.density.value)
            else:
                self.set_density(original_density * modifications.density.value)
        if modifications.mass:
            if modifications.mass.absolute:
                self.set_mass(modifications.mass.value)
            else:
                self.set_mass(original_mass * modifications.mass.value)
        self.update_inertia()
        self.modify_origin()

    def get_visual(self):
        """Returns the visual object of a link"""
        return self.element.visuals[0]

    def get_significant_length(self):
        """Gets the significant length for a cylinder or box geometry"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        if (geometry_type == geometry.Geometry.BOX):
            if (self.dimension is not None):
                if (self.dimension == geometry.Side.WIDTH):
                    return visual_data.size[0]
                elif (self.dimension == geometry.Side.HEIGHT):
                    return visual_data.size[1]
                elif (self.dimension == geometry.Side.DEPTH):
                    return visual_data.size[2]
            else:
                print(f"Error getting length for link {self.element.name}'s volume: Box geometry with no dimension")
        elif (geometry_type == geometry.Geometry.CYLINDER):
            return visual_data.length
        else:
            return None

    def get_radius(self):
        """Returns the radius if the link geometry is cylinder or sphere and None otherwise"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        return visual_data.radius if geometry_type == geometry.Geometry.CYLINDER or geometry_type == geometry.Geometry.SPHERE else None

    def set_radius(self, new_radius):
        """Sets the radius of a link if its geometry is cylider or sphere"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        if (geometry_type == geometry.Geometry.CYLINDER or geometry_type == geometry.Geometry.SPHERE):
            visual_data.radius = new_radius

    def set_length(self, length):
        """Modifies a link's length, in a manner that is logical with its geometry"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        if (geometry_type == geometry.Geometry.BOX):
            if (self.dimension is not None):
                if (self.dimension == geometry.Side.WIDTH):
                    visual_data.size[0] = length
                elif (self.dimension == geometry.Side.HEIGHT):
                    visual_data.size[1] = length
                elif (self.dimension == geometry.Side.DEPTH):
                    visual_data.size[2] = length
            else:
                print(f"Error modifying link {self.element.name}'s volume: Box geometry with no dimension")
        elif (geometry_type == geometry.Geometry.CYLINDER):
            visual_data.length = length

    @staticmethod
    def get_visual_static(link):
        """Static method that returns the visual of a link"""
        return link.visuals[0]

    @staticmethod
    def get_geometry(visual_obj):
        """Returns the geometry type and the corresponding geometry object for a given visual"""
        if (visual_obj.geometry.box is not None):
            return [geometry.Geometry.BOX, visual_obj.geometry.box]
        if (visual_obj.geometry.cylinder is not None):
            return [geometry.Geometry.CYLINDER, visual_obj.geometry.cylinder]
        if (visual_obj.geometry.sphere is not None):
            return [geometry.Geometry.SPHERE, visual_obj.geometry.sphere]

    def calculate_volume(self, geometry_type, visual_data):
        """Calculates volume with the formula that corresponds to the geometry"""
        if (geometry_type == geometry.Geometry.BOX):
            return visual_data.size[0] * visual_data.size[1] * visual_data.size[2]
        elif (geometry_type == geometry.Geometry.CYLINDER):
            return math.pi * visual_data.radius ** 2 * visual_data.length
        elif (geometry_type == geometry.Geometry.SPHERE):
            return 4 * math.pi * visual_data.radius ** 3 / 3

    def get_mass(self):
        """Returns the link's mass"""
        return self.element.inertial.mass

    def set_mass(self, new_mass):
        """Sets the mass value to a new value"""
        self.element.inertial.mass = new_mass

    def calculate_density(self):
        """Calculates density from mass and volume"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        return self.get_mass() / self.calculate_volume(geometry_type, visual_data)

    def modify_origin(self):
        """Modifies the position of the origin by a given amount"""
        visual_obj = self.get_visual()
        geometry_type, visual_data = self.get_geometry(visual_obj)
        xyz_rpy = matrix_to_xyz_rpy(visual_obj.origin)
        if (geometry_type == geometry.Geometry.BOX):
            if (self.dimension is not None):
                if (self.dimension == geometry.Side.WIDTH):
                    index_to_change = 0
                if (self.dimension == geometry.Side.HEIGHT):
                    index_to_change = 1
                if (self.dimension == geometry.Side.DEPTH):
                    index_to_change = 2
                if (self.calculate_origin_from_dimensions):
                    xyz_rpy[index_to_change] = (visual_data.size[index_to_change] if not self.flip_direction else -visual_data.size[index_to_change]) / 2
                xyz_rpy[index_to_change] += self.origin_modifier
                visual_obj.origin = xyz_rpy_to_matrix(xyz_rpy) 
            else:
                print(f"Error modifying link {self.element.name}'s origin: Box geometry with no dimension")
        elif (geometry_type == geometry.Geometry.CYLINDER):
            xyz_rpy[2] = -visual_data.length / 2 + self.origin_modifier
            visual_obj.origin = xyz_rpy_to_matrix(xyz_rpy)
        elif (geometry_type == geometry.Geometry.SPHERE):
            return

    def set_density(self, density):
        """Changes the mass of a link by preserving a given density."""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        volume = self.calculate_volume(geometry_type, visual_data)
        self.element.inertial.mass = volume * density

    def calculate_inertia(self):
        """Calculates inertia (ixx, iyy and izz) with the formula that corresponds to the geometry
        Formulas retrieved from https://en.wikipedia.org/wiki/List_of_moments_of_inertia"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        mass = self.get_mass()
        if (geometry_type == geometry.Geometry.BOX):
            return mass / 12 * np.array([visual_data.size[1] ** 2 + visual_data.size[2] ** 2, 
                                visual_data.size[0] ** 2 + visual_data.size[2] ** 2,
                                visual_data.size[0] ** 2 + visual_data.size[1] ** 2])
        elif (geometry_type == geometry.Geometry.CYLINDER):
            i_xy_incomplete = (3 * visual_data.radius ** 2 + visual_data.length ** 2) / 12
            return mass * np.array([i_xy_incomplete, i_xy_incomplete, visual_data.radius ** 2 / 2])
        elif (geometry_type == geometry.Geometry.SPHERE):
            inertia = 2 * mass * visual_data.radius ** 2 / 5
            return np.array([inertia, inertia, inertia])

    def update_inertia(self):
        """Updates the inertia of a link to match its volume and mass."""
        if (self.element.inertial is not None):
            inertia = self.element.inertial.inertia
            new_inertia = self.calculate_inertia()
            new_inertia[new_inertia < 0.01] = 0.01
            for i in range(3):
                for j in range(3):
                    if (i == j):
                        inertia[i,j] = new_inertia[i]
                    else:
                        inertia[i,j] = 0

    def __str__(self):
        return f"Link modifier with name {self.element.name}, origin modifier {self.origin_modifier}, dimension {self.dimension}"
