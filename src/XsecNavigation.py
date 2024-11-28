from enums import MotionCommand, DistanceType, Mission, Command
from dataclasses import dataclass
from duckietown_msgs.msg import WheelsCmdStamped
from geometry_msgs.msg import PoseStamped, Pose

#extensions
import math
import tf.transformations as tf
import numpy as np
    
#init params
WHEEL_DISTANCE = 0.102 #102
WHEEL_RADIUS = 0.035
TOL_CURVE = 0.05
TOL_ROTATE = 0.015
FIXED_SPEED = 0.25 #m/s   
FIXED_ANGULAR_SPEED = 0.25 #m/s
FIXED_ROTATION_SPEED = 0.25 #m/s

WHEEL_TOL = 0.05
    

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
                        direction=MotionCommand.Direction.POSITIVE, 
                        distance=np.pi/2,
                        radius=move_right_params["radius"]
                        ),
            ]
        )
        # self.move_right = Mission(
        #     name="move_right",
        #     commands=[
        #         Command(type=MotionCommand.Type.STRAIGHT, 
        #                 distance=move_right_params["distance"],
        #                 direction=MotionCommand.Direction.POSITIVE, 
        #                 ),
        #         Command(type=MotionCommand.Type.ROTATE, 
        #                 direction=MotionCommand.Direction.POSITIVE, 
        #                 distance=np.pi/2
        #                 ),
        #     ]
        # )
        self.move_left = Mission(
            name="move_left",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_left_params["distance"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
                Command(type=MotionCommand.Type.CURVE, 
                        direction=MotionCommand.Direction.NEGATIVE, 
                        distance=np.pi/2,
                        radius=move_left_params["radius"]
                        ),
            ]
        )
        # self.move_left = Mission(
        #     name="move_left",
        #     commands=[
        #         Command(type=MotionCommand.Type.STRAIGHT, 
        #                 distance=move_left_params["distance"],
        #                 direction=MotionCommand.Direction.POSITIVE, 
        #                 ),
        #         Command(type=MotionCommand.Type.ROTATE, 
        #                 direction=MotionCommand.Direction.NEGATIVE, 
        #                 distance=np.pi/2,
        #                 ),
        #     ]
        # )

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
            self.distance_current = 0
            self.counter = 0
            self.update_rate = update_rate
            self.dist_per_tick = 2*np.pi/resolution * WHEEL_RADIUS
            
            
            self.kp_l = 0.5
            self.kp_r = 0.5
            
            self.wheel_vel = FIXED_SPEED, FIXED_SPEED 
            self.goal_distance = [0, 0]
            self.current_distance = [0, 0]
            self.current_ticks = [0, 0]
            self.flag_goal_l = False
            self.flag_goal_r = False
        
        
        def get_wheel_cmd_ticks(self, ticks) -> WheelsCmdStamped:
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
                print("All done")
                wheel_cmd.vel_left = 0
                wheel_cmd.vel_right = 0
                self.all_commands_excecuted = True
                
            else:
                
                time = self.counter/self.update_rate
                self.current_ticks[0] = ticks[0] - self.init_tick[0]
                self.current_ticks[1] = ticks[1] - self.init_tick[1]
                print("current ticks: ", self.current_ticks, " time: ", time)
                    
                # Get the current command
                current_command = self.commands[self.current_command_index]
                command_type = current_command.type  
                direction = current_command.direction  
                distance = current_command.distance

                if command_type == MotionCommand.Type.STRAIGHT:
                    print("Straight")
                    
                    self.goal_distance[0] += FIXED_SPEED/self.update_rate 
                    self.goal_distance[1] += FIXED_SPEED/self.update_rate 
                    self.current_distance[0] = self.dist_per_tick * self.current_ticks[0]
                    self.current_distance[1] = self.dist_per_tick * self.current_ticks[1]
                    end_distance = [distance, distance]
                    dist_tol = 0.01

                elif command_type == MotionCommand.Type.ROTATE:
                    print("Rotate")
                    sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1
                    # Rotate the robot, either clockwise or counterclockwise
                    self.goal_distance[0] += sign * FIXED_SPEED/self.update_rate 
                    self.goal_distance[1] += -sign * FIXED_SPEED/self.update_rate 
                    self.current_distance[0] = self.dist_per_tick * self.current_ticks[0]
                    self.current_distance[1] = self.dist_per_tick * self.current_ticks[1]
                    end_distance = [sign * distance * WHEEL_DISTANCE/2, -sign * distance * WHEEL_DISTANCE/2] #distance from rad to m
                    dist_tol = 0.005
                       
                elif command_type == MotionCommand.Type.CURVE:
                    print("Curve")
                    sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1
                    # Move in a curve with a specified radius (TODO backwords curve)
                    radius = current_command.radius
                    radius_wheel = [0,0]
                    speed_wheel = [0,0]
                    radius_wheel[0] = radius + sign * WHEEL_DISTANCE / 2
                    radius_wheel[1] = radius - sign * WHEEL_DISTANCE / 2
                    speed_wheel[0] = FIXED_SPEED * (1 + sign * (WHEEL_DISTANCE / (2 * radius)))
                    speed_wheel[1] = FIXED_SPEED * (1 - sign * (WHEEL_DISTANCE / (2 * radius)))
                    
                    self.goal_distance[0] += speed_wheel[0]/self.update_rate
                    self.goal_distance[1] += speed_wheel[1]/self.update_rate 
                    self.current_distance[0] = self.dist_per_tick * self.current_ticks[0]
                    self.current_distance[1] = self.dist_per_tick * self.current_ticks[1]
                    end_distance = [distance * radius_wheel[0], distance * radius_wheel[1]] #distance from rad to m
                    dist_tol = 0.01
                    
                print("current distance: ", round(self.current_distance[0],3) , round(self.current_distance[1],3))
                print("Goal distance: ", round(self.goal_distance[0],3) , round(self.goal_distance[1],3))
                print("End distance: ", round(end_distance[0],3), round(end_distance[1],3))
                
                #control - switch between navigate and fine adjustment
                if self.flag_goal_r:
                    wheel_cmd.vel_left, wheel_cmd.vel_right = self.controller([self.goal_distance[0], end_distance[1]])
                elif self.flag_goal_l:
                    wheel_cmd.vel_left, wheel_cmd.vel_right = self.controller([end_distance[0], self.goal_distance[1]])
                else:
                    wheel_cmd.vel_left, wheel_cmd.vel_right = self.controller(self.goal_distance)
                
                #check if final position is reached for left and right
                if np.abs(self.current_distance[0] - end_distance[0]) < dist_tol:
                    wheel_cmd.vel_left = 0
                    self.flag_goal_l = True
                if np.abs(self.current_distance[1] - end_distance[1]) < dist_tol:
                    wheel_cmd.vel_right = 0
                    self.flag_goal_r = True

                #close command
                if self.flag_goal_r and self.flag_goal_l:
                    print("Command done")
                    #reinit
                    self.current_distance = [0, 0]
                    self.goal_distance = [0, 0]
                    self.counter = 0
                    self.init_tick = ticks
                    self.current_command_index += 1 
                    self.flag_goal_l = False
                    self.flag_goal_r = False
                else:
                    self.counter += 1

            return wheel_cmd
        
        def controller(self, target_distance):
            error_distance = [0,0]
            error_distance[0] = target_distance[0] - self.current_distance[0] #if (self.goal_distance[0] - self.current_distance[0]) >= 0 else 0 
            error_distance[1] = target_distance[1] - self.current_distance[1] #if (self.goal_distance[0] - self.current_distance[1]) >= 0 else 0 
            
            self.wheel_vel_l = self.kp_l * error_distance[0]
            self.wheel_vel_r = self.kp_r * error_distance[1]
            
            return self.wheel_vel_l, self.wheel_vel_r
        
        
        def get_wheel_cmd_pose(self, cur_pose: Pose) -> WheelsCmdStamped:
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

                if self.distance_current >= distance:
                    #reinit
                    self.initial_pose = cur_pose
                    self.distance_current = 0
                    self.current_command_index += 1
                    wheel_cmd.vel_left = 0
                    wheel_cmd.vel_right = 0

            return wheel_cmd
        
        def move_straight_feedback(self, direction, cur_pose):
            # Move in a straight line, either forward or backward
            speed = FIXED_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_SPEED
            
            #Get current distances
            self.distance_current = np.sqrt((np.abs(cur_pose.position.x) - np.abs(self.initial_pose.position.x))**2 + (np.abs(cur_pose.position.y) - np.abs(self.initial_pose.position.y))**2)
            current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
            print("Going Straight: current ", self.distance_current, " speed ", speed, ", yaw ", current_yaw)
            
            return speed, speed
        
        def rotate_feedback(self, direction, cur_pose):
            angular_speed = FIXED_ROTATION_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_ROTATION_SPEED   # Example speed for rotation
            
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
            self.distance_current = np.abs(self.initial_yaw - current_yaw) + TOL_ROTATE
            print("Rotate: current ", self.distance_current, " speed ", angular_speed, ", yaw ", current_yaw )

            return -angular_speed, angular_speed
        
        def move_on_curve_feedback(self, direction, cur_pose, radius):
            sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1 
            
            # get current and inital orientation
            current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
            self.distance_current = np.abs(self.wraptopi((current_yaw - self.initial_yaw))) + TOL_CURVE
            print("Making a curve: current ", self.distance_current, " speed ", FIXED_ANGULAR_SPEED)
    
            return FIXED_ANGULAR_SPEED * (1 - sign * (WHEEL_DISTANCE / (2 * radius))), FIXED_ANGULAR_SPEED * (1 + sign * (WHEEL_DISTANCE / (2 * radius)))
        
        def wraptopi(angle):
            #wraps to -pi to pi, also for multiturn
            angle = angle % (2*np.pi)
            if angle > np.pi:
                angle -= 2*np.pi
            else:
                angle += 2*np.pi
            return angle


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
                command_type = current_command.type.value  
                direction = current_command.direction
                distance = current_command.distance
                print(command_type, direction)
                if command_type == MotionCommand.Type.STRAIGHT:
                    print("Straight")
                    speed = FIXED_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_SPEED
                    wheel_cmd.vel_left, wheel_cmd.vel_right = speed, speed
                    
                    current_distance = time * speed

                elif command_type == MotionCommand.Type.ROTATE:
                    print("Rotate")
                    # Rotate the robot, either clockwise or counterclockwise
                    angular_speed = FIXED_ROTATION_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_ROTATION_SPEED   # Example speed for rotation
                    wheel_cmd.vel_left, wheel_cmd.vel_right = -angular_speed, angular_speed
                    
                    current_distance = angular_speed * (WHEEL_DISTANCE/2) * time
                    
                    
                elif command_type == MotionCommand.Type.CURVE:
                    print("Curve")
                    # Move in a curve with a specified radius
                    radius = current_command.radius 
                    sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1
                    wheel_cmd.vel_left = FIXED_SPEED * (1 + sign * (WHEEL_DISTANCE / (2 * radius))) 
                    wheel_cmd.vel_right = FIXED_SPEED * (1 - sign * (WHEEL_DISTANCE / (2 * radius)))
                    
                    avg_vel = (wheel_cmd.vel_right + wheel_cmd.vel_left)/2
                    
                    current_distance = avg_vel * time / radius
                    
                print("current distance: ", round(current_distance,3),"   Distance: ", round(distance,3))
                
                
                
                if current_distance >= distance:
                    print("Command done")
                    #reinit
                    current_distance = 0
                    self.current_command_index += 1
                    wheel_cmd.vel_left = 0
                    wheel_cmd.vel_right = 0
                    self.counter = 0
                else:
                    self.counter += 1
                    
            return wheel_cmd
        