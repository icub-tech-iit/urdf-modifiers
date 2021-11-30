from urdfpy import URDF
from urdfModifiers.geometry import *
from pathlib import Path
import argparse
import configparser
import os

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
    dimension_scale = config_section.get('dimension_scale', None)
    if dimension_scale is not None:
        modifications["dimension"] = [float(dimension_scale), False]
    density_scale = config_section.get('density_scale', None)
    if density_scale is not None:
        modifications["density"] = [float(density_scale), False]
    radius_scale = config_section.get('radius_scale', None)
    if radius_scale is not None:
        modifications["radius"] = [float(radius_scale), False]
    mass_scale = config_section.get('mass_scale', None)
    if mass_scale is not None:
        modifications["mass"] = [float(mass_scale), False]
    dimension = config_section.get('dimension', None)
    if dimension is not None:
        modifications["dimension"] = [float(dimension), True]
    density = config_section.get('density', None)
    if density is not None:
        modifications["density"] = [float(density), True]
    radius = config_section.get('radius', None)
    if radius is not None:
        modifications["radius"] = [float(radius), True]
    mass = config_section.get('mass', None)
    if mass is not None:
        modifications["mass"] = [float(mass), True]
    return modifications

def erase_dummy_file(dummy_filename):
    """Erases the dummy file"""
    os.remove(dummy_filename)

def install_urdf():
    """Uses Make to install the URDF in the build folder to the Gazebo path"""
    os.system("cd ../build && make install -j4")
