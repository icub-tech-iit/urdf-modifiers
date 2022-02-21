import unittest
import copy
from urdfModifiers.core.modification import Modification
from urdfModifiers.core.linkModifier import LinkModifier
from urdfModifiers.core.jointModifier import JointModifier
from urdfModifiers.geometry import *
from urdfModifiers.utils import *
from urdfpy import matrix_to_xyz_rpy 
import math

"""
Test Model:
root Link: world has 1 child(ren)
    child(1):  base_link -> sphere
        child(1):  aligned_link -> cylinder
            child(1):  connector_link_1 -> sphere
                child(1):  non_aligned_link -> box
                    child(1):  connector_link_2 -> sphere
"""

class LinkModifierTests(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(LinkModifierTests, self).__init__(*args, **kwargs)
        self.original_filename = './test_model.urdf'
        self.original_robot, _ = utils.load_robot_and_gazebo_plugins(self.original_filename, 'dummy.urdf')

    def setUp(self):
        self.modified_robot = copy.deepcopy(self.original_robot)

    def test_relative_mass_is_changed(self):

        modifier = LinkModifier.from_name('aligned_link', self.modified_robot)

        modification = Modification()
        modification.add_mass(2, absolute=False)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'aligned_link'][0]

        self.assertEqual(modified_link.inertial.mass, 
                         original_link.inertial.mass * 2)

    def test_absolute_mass_is_changed(self):

        modifier = LinkModifier.from_name('aligned_link', self.modified_robot)

        modification = Modification()
        modification.add_mass(2, absolute=True)

        modifier.modify(modification)

        modified_link = [link for link in self.modified_robot.links if link.name == 'aligned_link'][0]

        self.assertEqual(modified_link.inertial.mass, 
                         2)

    def test_relative_density_is_changed(self):

        modifier = LinkModifier.from_name('aligned_link', self.modified_robot)

        modification = Modification()
        modification.add_density(2, absolute=False)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'aligned_link'][0]

        self.assertEqual(modified_link.inertial.mass, 
                         original_link.inertial.mass * 2)
        # check geometry is equal
        self.assertEqual(modified_link.visuals[0].geometry.cylinder.radius, 
                         original_link.visuals[0].geometry.cylinder.radius)
        self.assertEqual(modified_link.visuals[0].geometry.cylinder.length, 
                         original_link.visuals[0].geometry.cylinder.length)

    def test_absolute_density_is_changed(self):

        modifier = LinkModifier.from_name('aligned_link', self.modified_robot)

        modification = Modification()
        modification.add_density(2, absolute=True)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'aligned_link'][0]

        radius = modified_link.visuals[0].geometry.cylinder.radius
        length = modified_link.visuals[0].geometry.cylinder.length
        volume = math.pi * length * (radius ** 2)

        self.assertEqual(modified_link.inertial.mass, 
                         volume * 2)
        # check geometry is equal
        self.assertEqual(modified_link.visuals[0].geometry.cylinder.radius, 
                         original_link.visuals[0].geometry.cylinder.radius)
        self.assertEqual(modified_link.visuals[0].geometry.cylinder.length, 
                         original_link.visuals[0].geometry.cylinder.length)

    def test_relative_radius_is_changed_cylinder(self):

        modifier = LinkModifier.from_name('aligned_link', self.modified_robot)

        modification = Modification()
        modification.add_radius(2, absolute=False)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.cylinder.radius, 
                         original_link.visuals[0].geometry.cylinder.radius * 2)

    def test_absolute_radius_is_changed_cylinder(self):

        modifier = LinkModifier.from_name('aligned_link', self.modified_robot)

        modification = Modification()
        modification.add_radius(2, absolute=True)

        modifier.modify(modification)

        modified_link = [link for link in self.modified_robot.links if link.name == 'aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.cylinder.radius, 
                         2)

    def test_relative_radius_is_changed_sphere(self):

        modifier = LinkModifier.from_name('base_link', self.modified_robot)

        modification = Modification()
        modification.add_radius(2, absolute=False)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'base_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'base_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.sphere.radius, 
                         original_link.visuals[0].geometry.sphere.radius * 2)

    def test_absolute_radius_is_changed_sphere(self):

        modifier = LinkModifier.from_name('base_link', self.modified_robot)

        modification = Modification()
        modification.add_radius(2, absolute=True)

        modifier.modify(modification)

        modified_link = [link for link in self.modified_robot.links if link.name == 'base_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.sphere.radius, 
                         2)

    def test_radius_fails_if_geometry_is_box(self):

        modifier = LinkModifier.from_name('non_aligned_link', self.modified_robot, dimension=geometry.Side.X)

        modification = Modification()
        modification.add_radius(2, absolute=True)

        with self.assertRaises(Exception) as context:
            modifier.modify(modification)

        self.assertTrue('Cannot modify radius of box geometry' in str(context.exception))

    def test_relative_length_is_changed_x(self):

        modifier = LinkModifier.from_name('non_aligned_link', self.modified_robot, dimension=geometry.Side.X)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'non_aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'non_aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.box.size[0], 
                         original_link.visuals[0].geometry.box.size[0] * 2)

        self.assertEqual(modified_link.visuals[0].geometry.box.size[1], 
                         original_link.visuals[0].geometry.box.size[1])

        self.assertEqual(modified_link.visuals[0].geometry.box.size[2], 
                         original_link.visuals[0].geometry.box.size[2])

        # check that origin is not being modified
        self.assertEqual(modified_link.visuals[0].origin.all(),
                         original_link.visuals[0].origin.all())
    
    def test_relative_length_is_changed_y(self):

        modifier = LinkModifier.from_name('non_aligned_link', self.modified_robot, dimension=geometry.Side.Y)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'non_aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'non_aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.box.size[0], 
                         original_link.visuals[0].geometry.box.size[0])

        self.assertEqual(modified_link.visuals[0].geometry.box.size[1], 
                         original_link.visuals[0].geometry.box.size[1] * 2)

        self.assertEqual(modified_link.visuals[0].geometry.box.size[2], 
                         original_link.visuals[0].geometry.box.size[2])

        # check that origin is not being modified
        self.assertEqual(modified_link.visuals[0].origin.all(),
                         original_link.visuals[0].origin.all())

    def test_relative_length_is_changed_z(self):

        modifier = LinkModifier.from_name('non_aligned_link', self.modified_robot, dimension=geometry.Side.Z)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'non_aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'non_aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.box.size[0], 
                         original_link.visuals[0].geometry.box.size[0])

        self.assertEqual(modified_link.visuals[0].geometry.box.size[1], 
                         original_link.visuals[0].geometry.box.size[1])

        self.assertEqual(modified_link.visuals[0].geometry.box.size[2], 
                         original_link.visuals[0].geometry.box.size[2] * 2)

        # check that origin is not being modified
        self.assertEqual(modified_link.visuals[0].origin.all(),
                         original_link.visuals[0].origin.all())

    def test_absolute_length_is_changed_x(self):

        modifier = LinkModifier.from_name('non_aligned_link', self.modified_robot, dimension=geometry.Side.X)

        modification = Modification()
        modification.add_dimension(2, absolute=True)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'non_aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'non_aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.box.size[0], 
                         2)

        self.assertEqual(modified_link.visuals[0].geometry.box.size[1], 
                         original_link.visuals[0].geometry.box.size[1])

        self.assertEqual(modified_link.visuals[0].geometry.box.size[2], 
                         original_link.visuals[0].geometry.box.size[2])

        # check that origin is not being modified
        self.assertEqual(modified_link.visuals[0].origin.all(),
                         original_link.visuals[0].origin.all())
    
    def test_absolute_length_is_changed_y(self):

        modifier = LinkModifier.from_name('non_aligned_link', self.modified_robot, dimension=geometry.Side.Y)

        modification = Modification()
        modification.add_dimension(2, absolute=True)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'non_aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'non_aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.box.size[0], 
                         original_link.visuals[0].geometry.box.size[0])

        self.assertEqual(modified_link.visuals[0].geometry.box.size[1], 
                         2)

        self.assertEqual(modified_link.visuals[0].geometry.box.size[2], 
                         original_link.visuals[0].geometry.box.size[2])

        # check that origin is not being modified
        self.assertEqual(modified_link.visuals[0].origin.all(),
                         original_link.visuals[0].origin.all())

    def test_absolute_length_is_changed_z(self):

        modifier = LinkModifier.from_name('non_aligned_link', self.modified_robot, dimension=geometry.Side.Z)

        modification = Modification()
        modification.add_dimension(2, absolute=True)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'non_aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'non_aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.box.size[0], 
                         original_link.visuals[0].geometry.box.size[0])

        self.assertEqual(modified_link.visuals[0].geometry.box.size[1], 
                         original_link.visuals[0].geometry.box.size[1])

        self.assertEqual(modified_link.visuals[0].geometry.box.size[2], 
                         2)

        # check that origin is not being modified
        self.assertEqual(modified_link.visuals[0].origin.all(),
                         original_link.visuals[0].origin.all())

    def test_relative_length_is_changed_cylinder(self):

        modifier = LinkModifier.from_name('aligned_link', self.modified_robot)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.cylinder.length, 
                         original_link.visuals[0].geometry.cylinder.length * 2)

        # check that origin is not being modified
        self.assertEqual(modified_link.visuals[0].origin.all(),
                         original_link.visuals[0].origin.all())

    def test_absolute_length_is_changed_cylinder(self):

        modifier = LinkModifier.from_name('aligned_link', self.modified_robot)

        modification = Modification()
        modification.add_dimension(2, absolute=True)

        modifier.modify(modification)

        original_link = [link for link in self.original_robot.links if link.name == 'aligned_link'][0]
        modified_link = [link for link in self.modified_robot.links if link.name == 'aligned_link'][0]

        self.assertEqual(modified_link.visuals[0].geometry.cylinder.length, 
                         2)

        # check that origin is not being modified
        self.assertEqual(modified_link.visuals[0].origin.all(),
                         original_link.visuals[0].origin.all())

    def test_dimension_fails_if_geometry_is_sphere(self):

        modifier = LinkModifier.from_name('base_link', self.modified_robot)

        modification = Modification()
        modification.add_dimension(2, absolute=True)

        with self.assertRaises(Exception) as context:
            modifier.modify(modification)

        self.assertTrue('Cannot modify length of sphere geometry' in str(context.exception))

    def test_fails_when_add_dimension_to_cylinder_link(self):

        with self.assertRaises(Exception) as context:
            modifier = LinkModifier.from_name('aligned_link', self.modified_robot, dimension=geometry.Side.Z)

        self.assertTrue('Modifier cannot have dimension for cylinder geometry' in str(context.exception))

    def test_fails_when_add_dimension_to_sphere_link(self):

        with self.assertRaises(Exception) as context:
            modifier = LinkModifier.from_name('base_link', self.modified_robot, dimension=geometry.Side.Z)

        self.assertTrue('Modifier cannot have dimension for sphere geometry' in str(context.exception))
    
    
class JointModifierTests(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(JointModifierTests, self).__init__(*args, **kwargs)
        self.original_filename = './test_model.urdf'
        self.original_robot, _ = utils.load_robot_and_gazebo_plugins(self.original_filename, 'dummy.urdf')

    def setUp(self):
        self.modified_robot = copy.deepcopy(self.original_robot)

    def test_relative_joint_change_x(self):

        modifier = JointModifier.from_name('non_aligned_link_joint_before', self.modified_robot, dimension=geometry.Side.X)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_joint = [joint for joint in self.original_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]
        modified_joint = [joint for joint in self.modified_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[0],
                         matrix_to_xyz_rpy(original_joint.origin)[0] * 2)

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[1],
                         matrix_to_xyz_rpy(original_joint.origin)[1])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[2],
                         matrix_to_xyz_rpy(original_joint.origin)[2])

    def test_relative_joint_change_x_from_0(self):

        modifier = JointModifier.from_name('joint_to_ground', self.modified_robot, dimension=geometry.Side.X)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_joint = [joint for joint in self.original_robot.joints if joint.name == 'joint_to_ground'][0]
        modified_joint = [joint for joint in self.modified_robot.joints if joint.name == 'joint_to_ground'][0]

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[0],
                         matrix_to_xyz_rpy(original_joint.origin)[0])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[1],
                         matrix_to_xyz_rpy(original_joint.origin)[1])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[2],
                         matrix_to_xyz_rpy(original_joint.origin)[2])

    def test_relative_joint_change_y(self):

        modifier = JointModifier.from_name('non_aligned_link_joint_before', self.modified_robot, dimension=geometry.Side.Y)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_joint = [joint for joint in self.original_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]
        modified_joint = [joint for joint in self.modified_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[0],
                         matrix_to_xyz_rpy(original_joint.origin)[0])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[1],
                         matrix_to_xyz_rpy(original_joint.origin)[1] * 2)

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[2],
                         matrix_to_xyz_rpy(original_joint.origin)[2])

    def test_relative_joint_change_y_from_0(self):

        modifier = JointModifier.from_name('joint_to_ground', self.modified_robot, dimension=geometry.Side.Y)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_joint = [joint for joint in self.original_robot.joints if joint.name == 'joint_to_ground'][0]
        modified_joint = [joint for joint in self.modified_robot.joints if joint.name == 'joint_to_ground'][0]

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[0],
                         matrix_to_xyz_rpy(original_joint.origin)[0])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[1],
                         matrix_to_xyz_rpy(original_joint.origin)[1])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[2],
                         matrix_to_xyz_rpy(original_joint.origin)[2])

    def test_relative_joint_change_z(self):

        modifier = JointModifier.from_name('non_aligned_link_joint_before', self.modified_robot, dimension=geometry.Side.Z)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_joint = [joint for joint in self.original_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]
        modified_joint = [joint for joint in self.modified_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[0],
                         matrix_to_xyz_rpy(original_joint.origin)[0])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[1],
                         matrix_to_xyz_rpy(original_joint.origin)[1])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[2],
                         matrix_to_xyz_rpy(original_joint.origin)[2] * 2)

    def test_relative_joint_change_z_from_0(self):

        modifier = JointModifier.from_name('joint_to_ground', self.modified_robot, dimension=geometry.Side.Z)

        modification = Modification()
        modification.add_dimension(2, absolute=False)

        modifier.modify(modification)

        original_joint = [joint for joint in self.original_robot.joints if joint.name == 'joint_to_ground'][0]
        modified_joint = [joint for joint in self.modified_robot.joints if joint.name == 'joint_to_ground'][0]

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[0],
                         matrix_to_xyz_rpy(original_joint.origin)[0])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[1],
                         matrix_to_xyz_rpy(original_joint.origin)[1])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[2],
                         matrix_to_xyz_rpy(original_joint.origin)[2])

    def test_absolute_joint_change_x(self):

        modifier = JointModifier.from_name('non_aligned_link_joint_before', self.modified_robot, dimension=geometry.Side.X)

        modification = Modification()
        modification.add_dimension(2, absolute=True)

        modifier.modify(modification)

        original_joint = [joint for joint in self.original_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]
        modified_joint = [joint for joint in self.modified_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[0],
                         2)

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[1],
                         matrix_to_xyz_rpy(original_joint.origin)[1])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[2],
                         matrix_to_xyz_rpy(original_joint.origin)[2])

    def test_absolute_joint_change_y(self):

        modifier = JointModifier.from_name('non_aligned_link_joint_before', self.modified_robot, dimension=geometry.Side.Y)

        modification = Modification()
        modification.add_dimension(2, absolute=True)

        modifier.modify(modification)

        original_joint = [joint for joint in self.original_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]
        modified_joint = [joint for joint in self.modified_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[0],
                         matrix_to_xyz_rpy(original_joint.origin)[0])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[1],
                         2)

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[2],
                         matrix_to_xyz_rpy(original_joint.origin)[2])

    def test_absolute_joint_change_z(self):

        modifier = JointModifier.from_name('non_aligned_link_joint_before', self.modified_robot, dimension=geometry.Side.Z)

        modification = Modification()
        modification.add_dimension(2, absolute=True)

        modifier.modify(modification)

        original_joint = [joint for joint in self.original_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]
        modified_joint = [joint for joint in self.modified_robot.joints if joint.name == 'non_aligned_link_joint_before'][0]

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[0],
                         matrix_to_xyz_rpy(original_joint.origin)[0])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[1],
                         matrix_to_xyz_rpy(original_joint.origin)[1])

        self.assertEqual(matrix_to_xyz_rpy(modified_joint.origin)[2],
                         2)

    def test_add_dimension_fails_if_no_dimension(self):

        modifier = JointModifier.from_name('aligned_link_joint_after', self.modified_robot)

        modification = Modification()
        modification.add_dimension(2, absolute=True)

        with self.assertRaises(Exception) as context:
            modifier.modify(modification)

        self.assertTrue('Dimension not specified for joint' in str(context.exception))

if __name__ == '__main__':
    unittest.main()
        
        