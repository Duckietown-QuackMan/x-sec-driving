from enums import MotionCommand, DistanceType, Mission, Command
from dataclasses import dataclass
from duckietown_msgs.msg import WheelsCmdStamped
from geometry_msgs.msg import PoseStamped, Pose

#extensions
import math
import tf.transformations as tf
import numpy as np
    
#init params
WHEEL_DISTANCE = 0.102
WHEEL_RADIUS = 0.035
TOL_CURVE = 0.005
TOL_ROTATE = 0.015
FIXED_SPEED = 0.02 #m/s   
FIXED_ANGULAR_SPEED = 0.02 #m/s
FIXED_ROTATION_SPEED = 0.02 #m/s
    

class XsecNavigator:
    
    def __init__(self, move_straight_params, move_right_params, move_left_params):
        
        self.move_straight = Mission(
            name="move_straight",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_straight_params["distance"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
            ]
        )
        self.move_right = Mission(
            name="move_right",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_right_params["distance"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
                Command(type=MotionCommand.Type.CURVE, 
                        direction=move_left_params["direction"], 
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
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
                Command(type=MotionCommand.Type.CURVE, 
                        direction=move_left_params["direction"], 
                        distance=np.pi/2,
                        radius=move_left_params["radius"]
                        ),
            ]
        )

    class Move():
        def __init__(self, initial_pose=[], commands=[], update_rate=1, init_ticks=(), resolution=135):
            """
            Initialize the Move class with initial pose and commands.
            Args:
                initial_pose (Pose): The initial pose of the robot.
                commands (list): A list of motion commands for the robot to execute.
            """
            
            self.commands = commands  # List of motion commands.
            self.current_command_index = 0  # Track which command is being executed.
            self.initial_pose = initial_pose
            self.init_tick = init_ticks #left, right 
            #self.initial_yaw = tf.euler_from_quaternion([self.initial_pose.orientation.x, self.initial_pose.orientation.y, self.initial_pose.orientation.z, self.initial_pose.orientation.w])[2]
            self.all_commands_excecuted = False
            self.distance_traveled = 0
            self.counter = 0
            self.update_rate = update_rate
            self.dist_per_tick = 2*np.pi/resolution * WHEEL_RADIUS
            
        def get_wheel_cmd(self) -> WheelsCmdStamped:
            """
            Calculate the wheel command based on the current pose and the target trajectory.
            Args:
                cur_pose (Pose): The current pose of the robot.
            Returns:
                WheelsCmdStamped: The command for the robot's wheels.
            """
            wheel_cmd = WheelsCmdStamped()
            time = self.counter/self.update_rate
            
            if self.current_command_index >= len(self.commands):
                # All commands have been executed, stop the robot
                print("All done")
                wheel_cmd.vel_left = 0
                wheel_cmd.vel_right = 0
                self.all_commands_excecuted = True
                
            else:
                # Get the current command
                current_command = self.commands[self.current_command_index]
                command_type = current_command.type  
                direction = current_command.direction  
                distance = current_command.distance

                if command_type == MotionCommand.Type.STRAIGHT:
                    print("Straight")
                    speed = FIXED_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_SPEED
                    wheel_cmd.vel_left, wheel_cmd.vel_right = speed, speed
                    
                    traveled_distance = time * FIXED_SPEED

                elif command_type == MotionCommand.Type.ROTATE:
                    print("Rotate")
                    # Rotate the robot, either clockwise or counterclockwise
                    angular_speed = FIXED_ROTATION_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_ROTATION_SPEED   # Example speed for rotation
                    wheel_cmd.vel_left, wheel_cmd.vel_right = -angular_speed, angular_speed
                    
                    traveled_distance = FIXED_ROTATION_SPEED * (WHEEL_DISTANCE/2) * time
                    
                    
                elif command_type == MotionCommand.Type.CURVE:
                    print("Curve")
                    # Move in a curve with a specified radius
                    radius = current_command.radius
                    sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1
                    wheel_cmd.vel_left = FIXED_ANGULAR_SPEED * (1 - sign * (WHEEL_DISTANCE / (2 * radius))) 
                    wheel_cmd.vel_right = FIXED_ANGULAR_SPEED * (1 + sign * (WHEEL_DISTANCE / (2 * radius)))
                    
                    avg_ang_vel = (wheel_cmd.vel_right + wheel_cmd.vel_left)/2
                    
                    traveled_distance = avg_ang_vel * time / radius
                    
                print("traveled distance: ", round(traveled_distance,3),"   Distance: ", round(distance,3))
                
                if traveled_distance >= distance:
                    print("Command done")
                    #reinit
                    traveled_distance = 0
                    self.current_command_index += 1
                    wheel_cmd.vel_left = 0
                    wheel_cmd.vel_right = 0
                    self.counter = 0
                else:
                    self.counter += 1
                    
            return wheel_cmd
        
        
        def get_wheel_cmd_ticks(self, ticks) -> WheelsCmdStamped:
            """
            Calculate the wheel command based on the current pose and the target trajectory.
            Args:
                cur_pose (Pose): The current pose of the robot.
            Returns:
                WheelsCmdStamped: The command for the robot's wheels.
            """
            wheel_cmd = WheelsCmdStamped()
            time = self.counter/self.update_rate
            avg_traveled_ticks = -0.1 * ((self.init_tick[0] + self.init_tick[1])/2 - (ticks[0] + ticks[1])/2)
            print("traveled ticks: ", avg_traveled_ticks)
            
            
            if self.current_command_index >= len(self.commands):
                # All commands have been executed, stop the robot
                print("All done")
                wheel_cmd.vel_left = 0
                wheel_cmd.vel_right = 0
                self.all_commands_excecuted = True
                
            else:
                # Get the current command
                current_command = self.commands[self.current_command_index]
                command_type = current_command.type  
                direction = current_command.direction  
                distance = current_command.distance

                if command_type == MotionCommand.Type.STRAIGHT:
                    print("Straight")
                    speed = FIXED_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_SPEED
                    wheel_cmd.vel_left, wheel_cmd.vel_right = speed, speed
                    
                    traveled_distance = self.dist_per_tick * avg_traveled_ticks

                elif command_type == MotionCommand.Type.ROTATE:
                    print("Rotate")
                    # Rotate the robot, either clockwise or counterclockwise
                    angular_speed = FIXED_ROTATION_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_ROTATION_SPEED   # Example speed for rotation
                    wheel_cmd.vel_left, wheel_cmd.vel_right = -angular_speed, angular_speed
                    
                    traveled_distance = self.dist_per_tick / (WHEEL_DISTANCE/2) * avg_traveled_ticks
                    
                    
                elif command_type == MotionCommand.Type.CURVE:
                    print("Curve")
                    # Move in a curve with a specified radius
                    radius = current_command.radius
                    sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1
                    wheel_cmd.vel_left = FIXED_ANGULAR_SPEED * (1 - sign * (WHEEL_DISTANCE / (2 * radius))) 
                    wheel_cmd.vel_right = FIXED_ANGULAR_SPEED * (1 + sign * (WHEEL_DISTANCE / (2 * radius)))
                    
                    traveled_distance = self.dist_per_tick / radius * avg_traveled_ticks
                    
                print("traveled distance: ", round(traveled_distance,3),"   Distance: ", round(distance,3))
                
                if traveled_distance >= distance:
                    print("Command done")
                    #reinit
                    traveled_distance = 0
                    self.current_command_index += 1
                    wheel_cmd.vel_left = 0
                    wheel_cmd.vel_right = 0
                    self.counter = 0
                else:
                    self.counter += 1

            return wheel_cmd
        
        
        def get_wheel_cmd_feedback(self, cur_pose: Pose) -> WheelsCmdStamped:
            """
            Calculate the wheel command based on the current pose and the target trajectory.
            Args:
                cur_pose (Pose): The current pose of the robot.
            Returns:
                WheelsCmdStamped: The command for the robot's wheels.
            """
            wheel_cmd = WheelsCmdStamped()
            
            if self.current_command_index >= len(self.commands):
                # All commands have been executed, stop the robot
                wheel_cmd.vel_left = 0
                wheel_cmd.vel_right = 0
                self.all_commands_excecuted = True
                
            else:
                # Get the current command
                current_command = self.commands[self.current_command_index]
                command_type = current_command.type  
                direction = current_command.direction  
                distance = current_command.distance

                if command_type == MotionCommand.Type.STRAIGHT:
                    wheel_cmd.vel_left, wheel_cmd.vel_right = self.move_straight(direction, cur_pose)

                elif command_type == MotionCommand.Type.ROTATE:
                    # Rotate the robot, either clockwise or counterclockwise
                    wheel_cmd.vel_left, wheel_cmd.vel_right = self.rotate(direction, cur_pose)
                    
                elif command_type == MotionCommand.Type.CURVE:
                    # Move in a curve with a specified radius
                    radius = current_command.radius
                    wheel_cmd.vel_left, wheel_cmd.vel_right = self.move_on_curve(direction, cur_pose, radius)

                if self.distance_traveled >= distance:
                    #reinit
                    self.initial_pose = cur_pose
                    self.distance_traveled = 0
                    self.current_command_index += 1
                    wheel_cmd.vel_left = 0
                    wheel_cmd.vel_right = 0

            return wheel_cmd
        
        
        def move_straight_feedback(self, direction, cur_pose):
            # Move in a straight line, either forward or backward
            speed = FIXED_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_SPEED
            
            #Get traveled distances
            self.distance_traveled = np.sqrt((np.abs(cur_pose.position.x) - np.abs(self.initial_pose.position.x))**2 + (np.abs(cur_pose.position.y) - np.abs(self.initial_pose.position.y))**2)
            current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
            print("Going Straight: traveled ", self.distance_traveled, " speed ", speed, ", yaw ", current_yaw)
            
            return speed, speed
        
        def rotate_feedback(self, direction, cur_pose):
            angular_speed = FIXED_ROTATION_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_ROTATION_SPEED   # Example speed for rotation
            
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
            self.distance_traveled = np.abs(self.initial_yaw - current_yaw) + TOL_ROTATE
            print("Rotate: traveled ", self.distance_traveled, " speed ", angular_speed, ", yaw ", current_yaw )

            return -angular_speed, angular_speed
        
        def move_on_curve_feedback(self, direction, cur_pose, radius):
            sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1 
            
            # get current and inital orientation
            current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
            self.distance_traveled = np.abs(self.wraptopi((current_yaw - self.initial_yaw))) + TOL_CURVE
            print("Making a curve: traveled ", self.distance_traveled, " speed ", FIXED_ANGULAR_SPEED)
    
            return FIXED_ANGULAR_SPEED * (1 - sign * (WHEEL_DISTANCE / (2 * radius))), FIXED_ANGULAR_SPEED * (1 + sign * (WHEEL_DISTANCE / (2 * radius)))
        
        def wraptopi(angle):
            #wraps to -pi to pi, also for multiturn
            angle = angle % (2*np.pi)
            if angle > np.pi:
                angle -= 2*np.pi
            else:
                angle += 2*np.pi
            return angle

