# urdf-modifiers
<img src="https://user-images.githubusercontent.com/56030908/144103210-44302013-2d99-4b5b-99c1-cd36fa906dde.png" width="900">


This library allows to modify a urdf kinematic chain links and joint and creating a new  urdf model out of it. 
The possible modifications are related to : 
- **Mass** (relative and absolute)
- **Density** (relative and absolute)
- **Dimension** (relative and absolute)
- **Radius** (relative and absolute)
- **Position** (relative and absolute)
- **Joint type** (_revolute_, _prismatic_, _continuous_, _fixed_)

An interface for reading the modifications from a configuration file is provided together with the library.

The library is aimed at modifying basic shapes, i.e. : 
- Sphere; 
- Cylinder; 
- Box.

If you want to convert your model into basic shapes, you can use [`idyntree-model-simplify-shapes`](https://github.com/robotology/idyntree#idyntree-model-simplify-shapes)
## :hammer: Dependencies

- [`python3`](https://wiki.python.org/moin/BeginnersGuide)

Other requisites are:

- [`urchin`](https://github.com/fishbotics/urchin)
- [`dataclasses`](https://pypi.org/project/dataclasses/)
- [`unittest`](https://pypi.org/project/unittest/)

They will be installed in the installation step!

## :floppy_disk: Installation

Install `python3`, if not installed (in **Ubuntu 20.04**):

```bash
sudo apt install python3.8
```

 Install the library via pip:

```bash

pip install urdf-modifiers  

```

preferably in a [virtual environment](https://docs.python.org/3/library/venv.html#venv-def). For example:

```bash
pip install virtualenv
python3 -m venv your_virtual_env
source your_virtual_env/bin/activate
```

## :rocket: Usage

### Link and Joint Modifier

These modifiers change the properties of the specific link/joint they are connected to. 

```python
from urdfModifiers.core.linkModifier import LinkModifier
from urdfModifiers.core.jointModifier import JointModifier
from urdfModifiers.core.modification import Modification
from urdfModifiers.utils import *
from urdfModifiers.geometry import *

urdf_path ="./models/stickBot/model.urdf"
output_file = "./models/stickBotModified.urdf"
dummy_file = 'no_gazebo_plugins.urdf'
# Extract the <gazebo> tags from the urdf, as they collide with the library
robot, gazebo_plugin_text = utils.load_robot_and_gazebo_plugins(urdf_path,dummy_file)

# Create a link modifier by specifying link name and robot
# axis parameter is necessary for dimension and position modifications
link_modifier = LinkModifier.from_name('r_upper_arm', robot, axis=geometry.Side.Z)
# Same applies for JointModifier
joint_modifier = JointModifier.from_name('r_elbow', robot, axis=geometry.Side.X)

# Create a new Modification instance
link_modifications = Modification()
# Make the link 0.2m long
link_modifications.add_dimension(0.2, absolute=True) 
# Double the link's density
link_modifications.add_density(2.0, absolute=False) 

# Create a new Modification instance 
joint_modifications = Modification()
# Move the origin of the joint to 0.4 in the modifier's axis
joint_modifications.add_position(0.4, absolute=True) 

# Apply the modifications
link_modifier.modify(link_modifications)
joint_modifier.modify(joint_modifications)

# Create a new Modification instance 
joint_modification_joint_type = Modification()
joint_modification_joint_type.add_joint_type(geometry.JointType.REVOLUTE)

# Apply the modification 
joint_modifier.modify(joint_modification_joint_type)

# Write URDF to a new file, also adding back the previously removed <gazebo> tags                
utils.write_urdf_to_file(robot, output_file, gazebo_plugin_text)  
```

### FixedOffsetModifier

This modifier is meant to be connected to a link. It will perform the same modifications as a simple `LinkModifier`. 

However, for **dimension** modifications it will also adjust the origin of both the link and its child joint (that is, the joint that comes immediately after this link in the URDF tree), so that the distance between the beginning of the link visual shape and its parent joint axis (**parent offset**), as well as the distance between the end of the link visual shape and its child joint axis (**child offset**) are preserved. You can see a diagram of this modification in the following image:

![FixedOffset Explanation](./assets/fomodifier.png)

```python
from urdfModifiers.core.fixedOffsetModifier import FixedOffsetModifier
from urdfModifiers.core.modification import Modification
from urdfModifiers.utils import *
from urdfModifiers.geometry import *

urdf_path ="./models/stickBot/model.urdf"
output_file = "./models/stickBotModified.urdf"
dummy_file = 'no_gazebo_plugins.urdf'
robot, gazebo_plugin_text = utils.load_robot_and_gazebo_plugins(urdf_path,dummy_file)

# FixedOffsetModifier does not need an axis since it assumes Z axis internally
fixed_offset_modifier = FixedOffsetModifier.from_name('r_upper_arm', robot)

fixed_offset_modifications = Modification()
# The following modification will behave the same as LinkModifier
fixed_offset_modifications.add_density(2.0, absolute=False) 
fixed_offset_modifications.add_radius(3.0, absolute=True)
# Dimension modifications will adjust the origins to keep the offsets
fixed_offset_modifications.add_dimension(0.2, absolute=True) 
# Position modifications are therefore ignored
# fixed_offset_modifications.add_position(3.0, absolute=False)

# Apply the modifications
fixed_offset_modifier.modify(fixed_offset_modifications)
             
utils.write_urdf_to_file(robot, output_file, gazebo_plugin_text) 
```

### From configuration file 

You can also create modifications from a `conf.ini` file.

```python

from urdfModifiers.core.linkModifier import LinkModifier
from urdfModifiers.core.jointModifier import JointModifier
from urdfModifiers.core.modification import Modification
from urdfModifiers.utils import *
import configparser

config_file_path = "./config/conf.ini"
config = configparser.ConfigParser()
config.read(config_file_path)

modifications = []

for config_section in config.sections():
    modifications.append(Modification.from_config_section(config[config_section]))
```

Here is an example of a valid `conf.ini` file:

```ini
[r_upper_arm]
dimension = 0.65
radius = 0.45
density_scale = 1.5

[r_lower_leg]
position_scale = 2.0

[l_ankle_roll]
joint_type = 'fixed'
```

The suffix `_scale` referes to relative modifications (so `absolute=False` when creating the modifier).

## Maintainers
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/Nicogene.png" width="40">](https://github.com/Nicogene) | [@Nicogene](https://github.com/Nicogene) |
| [<img src="https://github.com/mfussi66.png" width="40">](https://github.com/mfussi66) | [@mfussi66](https://github.com/mfussi66) |
