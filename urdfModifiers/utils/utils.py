from typing import Tuple
from urchin import URDF
from urdfModifiers.geometry import *
import os

def write_urdf_to_file(urdf, filename, gazebo_plugins=[]):
    """Saves the URDF to a valid .urdf file, also adding the gazebo_plugins"""
    urdf.save(filename)
    lines = []
    with open(filename, 'r') as f:
        lines = f.readlines()
        last_line = lines.pop()
        lines = lines + gazebo_plugins
        lines.append(last_line)

    with open(filename, 'w') as f:
        f.writelines(lines)

def separate_gazebo_plugins(filename):
    """Splits the URDF content in two parts: one relative to the robot and another to the gazebo plugins"""
    gazebo_lines = []
    robot_lines= []
    with open(filename, "r") as f:
        lines = f.readlines()
        is_gazebo = False
        for line in lines:
            if '<gazebo' in line or '< gazebo' in line:
                is_gazebo = True
            if (is_gazebo):
                gazebo_lines.append(line)
            else:
                robot_lines.append(line)
            if '</gazebo' in line or '< /gazebo' in line:
                is_gazebo = False
    return [robot_lines, gazebo_lines]

def create_dummy_file(dummy_filename, text):
    """Creates a dummy file to save information"""
    with open(dummy_filename, 'w') as f:
        f.writelines(text)

def erase_dummy_file(dummy_filename):
    """Erases the dummy file"""
    os.remove(dummy_filename)

def load_robot_and_gazebo_plugins(urdf_path:str, dummy_fileName:str)-> Tuple[URDF,str]:
    main_urdf, gazebo_plugin_text = separate_gazebo_plugins(urdf_path)
    create_dummy_file(dummy_fileName, main_urdf)
    robot = URDF.load(dummy_fileName)
    erase_dummy_file(dummy_fileName)
    return robot, gazebo_plugin_text