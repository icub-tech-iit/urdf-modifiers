from abc import ABCMeta, abstractmethod

class Modifier(metaclass=ABCMeta):
    """Class to contain information and methods on how to modify a URDF element"""
    def __init__(self, element, origin_modifier, element_type):
        self.element = element
        self.origin_modifier = origin_modifier
        self.element_type = element_type

    @abstractmethod
    def modify(self, modifications):
        pass

    @abstractmethod
    def modify_origin(self):
        pass
