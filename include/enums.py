from enum import Enum, IntEnum


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


class Shape(Enum):
    """
    Enumeration for different shapes the robot can draw.
    """
    SQUARE = 0
    CIRCLE = 1
    EIGHT = 2

    def __str__(self):
        """
        Return the string representation of the shape in lowercase.
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