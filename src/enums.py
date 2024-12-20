from enum import Enum, IntEnum
from typing import List, Union


class MotionCommand:
    """
    A class representing the type and direction of a motion command.
    """
    class Type(IntEnum):
        """
        Enumeration for different types of motion commands.
        """
        STRAIGHT = 0
        ROTATE = 1
        CURVE = 2

        def __str__(self):
            """
            Return the string representation of the motion type in lowercase.
            """
            return self.name.lower()

    class Direction(IntEnum):
        """
        Enumeration for the direction of motion.
        """
        POSITIVE = 0  # forward | left | counterclockwise
        NEGATIVE = 1  # backword | right | clockwise

        def __str__(self):
            """
            Return the string representation of the direction in lowercase.
            """
            return self.name.lower()
    
class Path(Enum):
    """ 
    Enumeration for different paths the robot takes at xsec
    """
    STRAIGHT = 0
    RIGHT = 1
    LEFT = 2
    
    def __str__(self):
        """
        Return the string representation of the shape in lowercase.
        """
        return self.name.lower()
      

class DistanceType(Enum):
    """
    Enumeration for the type of distance to track (distance or angle).
    """
    DISTANCE = 0
    ANGLE = 1

    def __str__(self):
        """
        Return the string representation of the distance type in lowercase.
        """
        return self.name.lower()
    
    
class Command:
    """ 
    Command defining parameters of a subpath.
    """
    
    def __init__(self, 
    type: MotionCommand.Type = MotionCommand.Type.STRAIGHT,
    direction: Union[MotionCommand.Direction, None] = None,
    distance: float = 0.0,
    radius: Union[float, None] = None):
    
        self.type = type
        self.direction = direction
        self.distance = distance
        self.radius = radius


class Mission:
    """
    Differnet missions.
    """
    def __init__ (self, name: str, commands: List[Command]):
        self.name = name
        self.commands = commands
        self.trajectory = None
