from urdfpy import URDF
from urdfModifiers.geometry import *
import os

from urdfModifiers.geometry.geometry import Modification

def write_urdf_to_file(urdf, filename, gazebo_plugins):
    """Saves the URDF to a valid .urdf file, also adding the gazebo_plugins"""
    urdf.save(filename)
    lines = ''
    with open(filename, 'r') as f:
        lines = f.readlines()
        lines.pop()
        lines = lines + gazebo_plugins

    with open(filename, 'w') as f:
        f.writelines(lines)

def separate_gazebo_plugins(filename):
    """Splits the URDF content in two parts: one relative to the robot and another to the gazebo plugins"""
    gazebo_lines = []
    other_lines= []
    with open(filename, "r") as f:
        lines = f.readlines()
        is_gazebo = False
        for line in lines:
            if '<gazebo' in line or '< gazebo' in line:
                is_gazebo = True
            if (is_gazebo):
                gazebo_lines.append(line)
            else:
                other_lines.append(line)
    other_lines.append('</robot>')
    return [other_lines, gazebo_lines]

def create_dummy_file(dummy_filename, text):
    """Creates a dummy file to save information"""
    with open(dummy_filename, 'w') as f:
        f.writelines(text)

def parse_modifications(config_section):
    """Returns the modifications from a section of the ini file"""
    modifications = {}
    dimension_scale = config_section.get(geometry.Modification.DIMENSION+geometry.Modification.SCALE, None)
    if dimension_scale is not None:
        modifications[geometry.Modification.DIMENSION] = [float(dimension_scale), geometry.Modification.MULTIPLIER]
    density_scale = config_section.get(geometry.Modification.DENSITY+geometry.Modification.SCALE, None)
    if density_scale is not None:
        modifications[geometry.Modification.DENSITY] = [float(density_scale), geometry.Modification.MULTIPLIER]
    radius_scale = config_section.get(geometry.Modification.DENSITY+Modification.SCALE, None)
    if radius_scale is not None:
        modifications[geometry.Modification.RADIUS] = [float(radius_scale), geometry.Modification.MULTIPLIER]
    mass_scale = config_section.get(geometry.Modification.MASS+geometry.Modification.SCALE, None)
    if mass_scale is not None:
        modifications[geometry.Modification.MASS] = [float(mass_scale), geometry.Modification.MULTIPLIER]
    dimension = config_section.get(geometry.Modification.DIMENSION, None)
    if dimension is not None:
        modifications[geometry.Modification.DIMENSION] = [float(dimension), geometry.Modification.ABSOLUTE]
    density = config_section.get(geometry.Modification.DENSITY, None)
    if density is not None:
        modifications[geometry.Modification.DENSITY] = [float(density), geometry.Modification.ABSOLUTE]
    radius = config_section.get(geometry.Modification.RADIUS, None)
    if radius is not None:
        modifications[geometry.Modification.RADIUS] = [float(radius), geometry.Modification.ABSOLUTE]
    mass = config_section.get(geometry.Modification.MASS, None)
    if mass is not None:
        modifications[geometry.Modification.MASS] = [float(mass), geometry.Modification.ABSOLUTE]
    return modifications

def erase_dummy_file(dummy_filename):
    """Erases the dummy file"""
    os.remove(dummy_filename)

# TODO is returning a string 
def load_robot_and_gazebo_plugins(urdf_path,dummy_fileName): 
    main_urdf, gazebo_plugin_text = separate_gazebo_plugins(urdf_path)
    create_dummy_file(dummy_fileName, main_urdf)
    robot = URDF.load(dummy_fileName)
    erase_dummy_file(dummy_fileName)
    return robot, gazebo_plugin_text