from dataclasses import dataclass

@dataclass
class ModificationType:
    """Standard class to describe a specific type of modification"""
    value = None
    absolute = False

    def __init__(self, value, absolute):
        self.value = value
        self.absolute = absolute

@dataclass
class Modification:
    """Class to describe the modifications to perform to a link"""    
    def __init__(self):
        self.mass = None
        self.density = None
        self.dimension = None
        self.radius = None
        self.position = None
        self.joint_type = None 
        self.offset_mask = [1,1,1]
        pass

    @classmethod
    def from_config_section(cls, config_section):
        """Constructs a modification from a section of a configparser.ConfigParser class"""
        new_modification = cls()

        dimension_scale = config_section.get('dimension_scale', None)
        if dimension_scale is not None:
            new_modification.add_dimension(float(dimension_scale), False)
        dimension = config_section.get('dimension', None)
        if dimension is not None:
            new_modification.add_dimension(float(dimension), True)
        
        density_scale = config_section.get('density_scale', None)
        if density_scale is not None:
            new_modification.add_density(float(density_scale), False)
        density = config_section.get('density', None)
        if density is not None:
            new_modification.add_density(float(density), True)
        
        mass_scale = config_section.get('mass_scale', None)
        if mass_scale is not None:
            new_modification.add_mass(float(mass_scale), False)
        mass = config_section.get('mass', None)
        if mass is not None:
            new_modification.add_mass(float(mass), True)

        radius_scale = config_section.get('radius_scale', None)
        if radius_scale is not None:
            new_modification.add_radius(float(radius_scale), False)
        radius = config_section.get('radius', None)
        if radius is not None:
            new_modification.add_radius(float(radius), True)

        position_scale = config_section.get('position_scale', None)
        if position_scale is not None:
            new_modification.add_position(float(position_scale), False)
        position = config_section.get('position', None)
        if position is not None:
            new_modification.add_position(float(position), True)

        joint_type_modification = config_section.get('joint_type', None)

        if(joint_type_modification): 
            new_modification.add_joint_type(str(joint_type_modification))

        return new_modification

    def add_density(self, value, absolute):
        """Adds a modification of the density"""
        self.density = ModificationType(value, absolute)

    def add_dimension(self, value, absolute, offset_mask=None):
        """Adds a modification of the main dimension"""
        self.dimension = ModificationType(value, absolute)
        if offset_mask is not None:
            self.add_offset_mask(offset_mask)

    def add_offset_mask(self, offset_mask):
        """Changes the offset mask for choosing which offsets to keep.
        Mask should be an array of 3 truthy/falsy values representing each axis"""
        if not isinstance(offset_mask, list) or len(offset_mask) != 3:
            raise Exception("Invalid Offset Mask parameter, expected array with 3 values")
        self.offset_mask = offset_mask

    def add_mass(self, value, absolute):
        """Adds a modification of the mass"""
        self.mass = ModificationType(value, absolute)

    def add_radius(self, value, absolute):
        """Adds a modification of the radius"""
        self.radius = ModificationType(value, absolute)

    def add_position(self, value, absolute):
        """Adds a modification of the position of the origin"""
        self.position = ModificationType(value, absolute)

    def add_joint_type(self, value):
        """Adds a modification of the type of joint (revolute, fixed, etc)"""
        self.joint_type = value

    def __str__(self):
        print_message = "Modification class with the following parameters: "
        if self.mass:
            print_message += f"{'Absolute' if self.mass.absolute else 'Relative'} modification of mass with value {self.mass.value}. "
        if self.density:
            print_message += f"{'Absolute' if self.density.absolute else 'Relative'} modification of density with value {self.density.value}. "
        if self.dimension:
            print_message += f"{'Absolute' if self.dimension.absolute else 'Relative'} modification of dimension with value {self.dimension.value}. "
        if self.radius:
            print_message += f"{'Absolute' if self.radius.absolute else 'Relative'} modification of radius with value {self.radius.value}. "
        if self.position:
            print_message += f"{'Absolute' if self.position.absolute else 'Relative'} modification of origin position with value {self.position.value}."
        if self.joint_type: 
            print_message += f"Joint Type "+ self.joint_type
        return print_message
        