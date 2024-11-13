from include.enums import MotionCommand, DistanceType, Mission, Command
from dataclasses import dataclass
from duckietown_msgs.msg import WheelsCmdStamped
from geometry_msgs.msg import PoseStamped, Pose

#extensions
import math
import tf.transformations as tf
import numpy as np

@dataclass
class Distance:
    distance_type: DistanceType
    value: float

class XsecNavigator:
    def __init__(self, move_straight_params, move_right_params, move_left_params):
        
        #init params
        self.wheel_distance = 0.102  # Distance between the left and right wheels.
        self.tol_curve = 0.005
        self.tol_rotate = 0.015
        
        
        self.move_straight = Mission(
            name="move_straight",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_straight_params["distance"],
                        ),
            ]
        )
        self.move_right = Mission(
            name="move_right",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_right_params["distance"],
                        ),
                Command(type=MotionCommand.Type.CURVE, 
                        direction=MotionCommand.Direction.POSITIVE, 
                        distance=np.pi/2,
                        radius=move_right_params["radius"]
                        ),
            ]
        )
        self.move_left = Mission(
            name="move_left",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_left_params["distance"],
                        ),
                Command(type=MotionCommand.Type.CURVE, 
                        direction=MotionCommand.Direction.NEGATIVE, 
                        distance=np.pi/2,
                        radius=move_left_params["radius"]
                        ),
            ]
        )


    class Move:
        def __init__(self, initial_pose, commands=[]):
            """
            Initialize the Move class with initial pose and commands.
            Args:
                initial_pose (Pose): The initial pose of the robot.
                commands (list): A list of motion commands for the robot to execute.
            """
            
            self.commands = commands  # List of motion commands.
            self.current_command_index = 0  # Track which command is being executed.
            self.initial_pose = initial_pose
            self.all_commands_excecuted = False
            self.value_traveled = 0
            self.value_idx = 0
            
            

        def get_wheel_cmd(self, cur_pose: Pose) -> WheelsCmdStamped:
            """
            Calculate the wheel command based on the current pose and the target trajectory.
            Args:
                cur_pose (Pose): The current pose of the robot.
            Returns:
                WheelsCmdStamped: The command for the robot's wheels.
            """
            if self.current_command_index >= len(self.commands):
                # All commands have been executed, stop the robot
                wheel_cmd = WheelsCmdStamped()
                wheel_cmd.vel_left = 0
                wheel_cmd.vel_right = 0
                self.all_commands_excecuted = True
                return wheel_cmd
            

            # Get the current command
            current_command = self.commands[self.current_command_index]
            command_type = current_command[0]  # MotionCommand.Type
            direction = current_command[1]  # MotionCommand.Direction
            value = current_command[2]  # Distance for STRAIGHT, angle for ROTATE, radius for CURVE

            wheel_cmd = WheelsCmdStamped()

            if command_type == MotionCommand.Type.STRAIGHT:
                self.value_idx = 1 #no multiturn possible 
                # Move in a straight line, either forward or backward
                if direction == MotionCommand.Direction.POSITIVE:
                    speed = 0.2 
                else:
                    speed = -0.2
                    
                wheel_cmd.vel_left = speed
                wheel_cmd.vel_right = speed
                self.value_traveled = np.sqrt((np.abs(cur_pose.position.x) - np.abs(self.initial_pose.position.x))**2 + (np.abs(cur_pose.position.y) - np.abs(self.initial_pose.position.y))**2)
                current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
                print("Going Straight: traveled ", self.value_traveled, " speed ", speed, ", yaw ", current_yaw)

            elif command_type == MotionCommand.Type.ROTATE:
                # Rotate the robot, either clockwise or counterclockwise
                angular_speed = 0.05 * math.pi / 4  # Example speed for rotation
                if direction == MotionCommand.Direction.POSITIVE:
                    # Counterclockwise rotation
                    wheel_cmd.vel_left = -angular_speed
                    wheel_cmd.vel_right = angular_speed
                else:
                    # Clockwise rotation
                    wheel_cmd.vel_left = angular_speed
                    wheel_cmd.vel_right = -angular_speed
                
                #logic to stop after rotating the required angle
                # Convert quaternion to Euler angles (roll, pitch, yaw)
                initial_yaw = tf.euler_from_quaternion([self.initial_pose.orientation.x, self.initial_pose.orientation.y, self.initial_pose.orientation.z, self.initial_pose.orientation.w])[2]
                current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
                self.value_traveled = np.abs(initial_yaw - current_yaw) + self.tol_rotate
                print("Rotate: traveled ", self.value_traveled, " speed ", angular_speed, ", yaw ", current_yaw )

            elif command_type == MotionCommand.Type.CURVE:
                # Move in a curve with a specified radius
                radius = current_command[3]   
                angular_speed = 0.5  # Example angular speed for the curve
                # Logic to stop after moving along the required curve distance
                initial_yaw = tf.euler_from_quaternion([self.initial_pose.orientation.x, self.initial_pose.orientation.y, self.initial_pose.orientation.z, self.initial_pose.orientation.w])[2] 
                current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
                            
                print(initial_yaw, current_yaw)
                
                if direction == MotionCommand.Direction.POSITIVE:
                    wheel_cmd.vel_left = angular_speed * (1 - (self.wheel_distance / (2 * radius)))
                    wheel_cmd.vel_right = angular_speed * (1 + (self.wheel_distance / (2 * radius)))
                    
                    #translate yaw angle
                    current_yaw -= initial_yaw
                    initial_yaw = 0
                    #rotate yaw angle
                    current_yaw = current_yaw + 2*np.pi * (current_yaw < 0)
                    
                else:
                    wheel_cmd.vel_left = angular_speed * (1 + (self.wheel_distance / (2 * radius)))
                    wheel_cmd.vel_right = angular_speed * (1 - (self.wheel_distance / (2 * radius)))
                    
                    #translate yaw angle
                    current_yaw = -1 * (current_yaw - initial_yaw)
                    initial_yaw = 0
                    #rotate yaw angle
                    current_yaw = current_yaw + 2*np.pi * (current_yaw < 0)

                self.value_traveled = np.abs(current_yaw - initial_yaw) + self.tol_curve
                
                print("Making a curve: traveled ", self.value_traveled, " speed ", angular_speed)

            # mutltiturn conditioning
            if self.value_idx == 0: 
                self.value_idx = math.ceil(value / (2*np.pi))
            elif self.value_idx == 1 or value == 0: 
                # finished command 
                if self.value_traveled >= value:
                    self.initial_pose = cur_pose
                    self.value_idx = 0 
                    self.value_traveled = 0

                    self.current_command_index += 1
                    wheel_cmd.vel_left = 0
                    wheel_cmd.vel_right = 0
            else:
                self.value_idx -= 1


            return wheel_cmd