# urdf-modifiers
<img src="https://user-images.githubusercontent.com/56030908/144103210-44302013-2d99-4b5b-99c1-cd36fa906dde.png" width="900">


This library allows to modify a urdf kinematic chain links and joint and creating a new  urdf model out of it. 
The possible modifications are related to : 
- **Mass** (relative and absolute);
- **Density** (relative and absolute);
- **Dimension** (relative and absolute);
- **Radius** (relative and absolute). 

An interface for reading the modifications from a configuration file is provided together with the library.

The library is aimed at modifying basic shapes, i.e. : 
- Sphere; 
- Cylinder; 
- Box.

If you want to convert your model into basic shapes, you can use [`idyntree-model-simplify-shapes`](https://github.com/robotology/idyntree#idyntree-model-simplify-shapes)
## :hammer: Dependencies

- [`python3`](https://wiki.python.org/moin/BeginnersGuide)

Other requisites are:

- [`urdfpy`](https://github.com/mmatl/urdfpy)
- [`dataclasses`](https://pypi.org/project/dataclasses/)

They will be installed in the installation step!

## :floppy_disk: Installation

Install `python3`, if not installed (in **Ubuntu 20.04**):

```bash
sudo apt install python3.8
```

Clone the repo and install the library:

```bash

pip install "urdfModifiers @ git+https://github.com/icub-tech-iit/urdf-modifiers"

```

preferably in a [virtual environment](https://docs.python.org/3/library/venv.html#venv-def). For example:

```bash
pip install virtualenv
python3 -m venv your_virtual_env
source your_virtual_env/bin/activate
```

## :rocket: Usage

### From script

```python
from urdfModifiers.core.linkModifier import LinkModifier
from urdfModifiers.core.jointModifier import JointModifier
from urdfModifiers.utils import *

urdf_path ="./models/stickBot/model.urdf"
output_file = "./models/stickBotModified.urdf"
dummy_file = 'no_gazebo_plugins.urdf'
robot, gazebo_plugin_text = utils.load_robot_and_gazebo_plugins(urdf_path,dummy_file)

modifications= {}
modifications[utils.geometry.Modification.DIMENSION] = [0.2, utils.geometry.Modification.ABSOLUTE] # Makes the link 0.2m long
modifications[utils.geometry.Modification.DENSITY] = [2.0, utils.geometry.Modification.MULTIPLIER] # Doubles the link's density

modifiers = [LinkModifier.from_name('r_upper_arm',robot, 0.022),
                JointModifier.from_name('r_elbow',robot, 0.0344)]
                
for item in modifiers:
    item.modify(modifications)
utils.write_urdf_to_file(robot, output_file, gazebo_plugin_text)  
```

### From configuration file 

```python

from urdfModifiers.core.linkModifier import LinkModifier
from urdfModifiers.core.jointModifier import JointModifier
from urdfModifiers.utils import *
import configparser

urdf_path ="./models/stickBot/model.urdf"
output_file = "./models/stickBotModified.urdf"
dummy_file = 'no_gazebo_plugins.urdf'

robot, gazebo_plugin_text = utils.load_robot_and_gazebo_plugins(urdf_path,dummy_file)

config_file_path = "./config/conf.ini"
config = configparser.ConfigParser()
config.read(config_file_path)

for config_section in config.sections():
    modifications = utils.parse_modifications(config[config_section])
    selector = config_section
    if(selector == 'r_upper_arm'):
        modifiers = [LinkModifier.from_name('r_upper_arm',robot, 0.022),
        JointModifier.from_name('r_elbow',robot, 0.0344)]
                        
for item in modifiers:
    item.modify(modifications)
utils.write_urdf_to_file(robot, output_file, gazebo_plugin_text)

```
