from enums import MotionCommand, DistanceType, Mission, Command
from dataclasses import dataclass
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from geometry_msgs.msg import PoseStamped, Pose
import matplotlib.pyplot as plt
import rospy

#extensions
import math
import tf.transformations as tf
import numpy as np
    
#init params
WHEEL_DISTANCE = 0.102 #102
WHEEL_RADIUS = 0.035
TOL_CURVE = 0.05
TOL_ROTATE = 0.015
FIXED_SPEED = 0.2 #m/s   
FIXED_ANGULAR_SPEED = 6 
FIXED_ROTATION_SPEED = 6 
BOT_CORRECTION = 0.7  #gimpy

    

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
                        distance=move_right_params["distance"] + 0.15,
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
                Command(type=MotionCommand.Type.ROTATE, 
                        direction=MotionCommand.Direction.POSITIVE, 
                        distance=np.pi/2,
                        ),
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_right_params["distance"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
            ]
        )
        

        self.move_left = Mission(
            name="move_left",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_left_params["distance"] + 0.15,
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
                Command(type=MotionCommand.Type.ROTATE, 
                        direction=MotionCommand.Direction.NEGATIVE, 
                        distance=np.pi/2,
                        ),
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_left_params["distance"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
            ]
        )
        
        self.trajectories = self.create_trajectory()
    
    def create_trajectory(self):
        
        missions = [self.move_straight, self.move_left, self.move_right]
        trajectories = []
    
        for mission in missions:
            
            trajectory = []
            sample_dist_m = 0.025 
            sample_dist_rad = 0.2
            self.commands = mission.commands  # List of motion commands.
            self.current_command_index = 0  # Track which command is being executed.
            x_start = 0
            y_start = 0 
                    
            # Get the current command
            for current_command in self.commands:
                
                command_type = current_command.type  
                direction = current_command.direction  
                distance = current_command.distance
                
                if command_type == MotionCommand.Type.STRAIGHT:
                    num_samples = int(distance/sample_dist_m)
                    t = np.linspace(0, distance, num_samples)[1:]
                    x_coord = np.zeros(t.size)
                    y_coord = t
                elif command_type == MotionCommand.Type.ROTATE:
                    num_samples = int(distance/sample_dist_rad)
                    t = np.linspace(0, distance, num_samples)[1:]
                    x_coord = np.zeros(t.size)
                    y_coord = np.zeros(t.size)
                elif command_type == MotionCommand.Type.CURVE:
                    num_samples = int(distance/sample_dist_rad)
                    t = np.linspace(0, distance, num_samples)[1:]
                    sign = -1 if direction == MotionCommand.Direction.POSITIVE else 1
                    radius = current_command.radius
                    x_coord = radius * sign * (np.cos(t) - 1)
                    y_coord = radius * np.sin(t)
                
                #shift coordinates 
                x_coord += x_start
                y_coord += y_start
                #reinit 
                x_start = x_coord[-1]
                y_start = y_coord[-1]
                
                trajectory.extend(list(zip(x_coord, y_coord)))
                #rospy.loginfo(trajectory)
    
            #self.plot_trajectory(trajectory)
            trajectories.append(trajectory)
        
        return trajectories
            
    def plot_trajectory(self, trajectory):
         
        # Plot original curve and equidistant points
        x, y = zip(*trajectory)  # Unpack points for plotting
        plt.figure(figsize=(8, 6))
        plt.plot(x, y, label="Original Curve", alpha=0.6)
        plt.scatter(x, y, color="red", label="Equidistant Points", zorder=5)
        plt.legend()
        plt.axis("equal")
        plt.title("Equidistant Points on a Curve")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.show()

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
            self.radius_wheel = [0,0]
            self.flag_done = False
            self.flag_l = False
            self.flag_r = False
            self.fine_adjust = False
            
            #traj
            self.current_pose = (0, 0, 0)
            
        ################################
        ### controller using tick feedback
        ################################
        
        def get_wheel_cmd_ticks(self, ticks):
            """
            Calculate the wheel command based on the current pose and the target trajectory.
            Args:
                cur_pose (Pose): The current pose of the robot.
            Returns:
                WheelsCmdStamped: The command for the robot's wheels.
            """
            wheel_cmd = Twist2DStamped()
            wheel_adj = WheelsCmdStamped()
       
            if self.current_command_index >= len(self.commands):
                # All commands have been executed, stop the robot
                rospy.loginfo(f"All done")
                wheel_cmd.v = 0
                wheel_cmd.omega = 0
                wheel_adj.vel_left = 0
                wheel_adj.vel_left = 0
                self.all_commands_excecuted = True
                
            else:
                
                if self.flag_done:
                    self.init_tick = ticks
                    self.flag_done = False
                
                self.current_ticks[0] = ticks[0] - self.init_tick[0]
                self.current_ticks[1] = ticks[1] - self.init_tick[1]
                self.current_distance[0] = self.dist_per_tick * self.current_ticks[0]
                self.current_distance[1] = self.dist_per_tick * self.current_ticks[1]
                    
                # Get the current command
                current_command = self.commands[self.current_command_index]
                command_type = current_command.type  
                direction = current_command.direction  
                distance = current_command.distance
                
                if command_type == MotionCommand.Type.STRAIGHT:
                    end_distance = [distance, distance]
                    wheel_cmd.v = FIXED_SPEED
                    wheel_cmd.omega = 0
                    rospy.loginfo(f"Straight")
                    rospy.loginfo(f"{end_distance}, {self.current_distance}")
                    
                
                elif command_type == MotionCommand.Type.ROTATE:
                    rospy.loginfo(f"Rotate")
                    # Rotate the robot, either clockwise or counterclockwise
                    sign = 1 * BOT_CORRECTION if direction == MotionCommand.Direction.POSITIVE else -1 
                    end_distance = [sign * distance * WHEEL_DISTANCE/2, -sign * distance * WHEEL_DISTANCE/2] #distance from rad to m
                    wheel_cmd.v = 0
                    wheel_cmd.omega = -sign * FIXED_ANGULAR_SPEED
                    
                    rospy.loginfo(f"{end_distance} , {self.current_distance}")
                    rospy.loginfo(f"current ticks: {self.current_ticks}")
                       
                elif command_type == MotionCommand.Type.CURVE:
                    #rospy.loginfo("Curve")
                    # Move in a curve with a specified radius (TODO backwords curve)
                    sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1
                    radius = current_command.radius
                    self.radius_wheel[0] = radius + sign * WHEEL_DISTANCE / 2
                    self.radius_wheel[1] = radius - sign * WHEEL_DISTANCE / 2
                    end_distance = [distance * self.radius_wheel[0], distance * self.radius_wheel[1]] #distance from rad to m
                    wheel_cmd.v = FIXED_SPEED
                    wheel_cmd.omega = FIXED_SPEED/radius 
                    
                    
                       
                #--fineadjustment
                #check if goal reached
                if (self.flag_l or self.flag_r):
                    wheel_cmd.v = 0
                    wheel_cmd.omega = 0
                    wheel_adj.vel_right = 0
                    wheel_adj.vel_left = 0
                    rospy.sleep(0.1)
                    #reinit
                    self.current_distance = [0, 0]
                    self.flag_done = True
                    self.flag_r = False 
                    self.flag_l = False
                    self.fine_adjust = False
                    self.current_command_index += 1
                    
                  
                #check if final position is reached for left wheel
                elif np.abs(self.current_distance[0]) > np.abs(end_distance[0]):
                    self.fine_adjust = True
                    rospy.loginfo(f"{end_distance} , {self.current_distance}")
                    rospy.loginfo(f"current ticks, left reached: {self.current_ticks}")
                    wheel_cmd.v = 0
                    wheel_cmd.omega = 0
                    #tick difference
                    tickdiff_l = np.abs(self.current_ticks[0]) - np.abs(self.current_ticks[1])
                    #check if wheels reached similar distance
                    if np.abs(tickdiff_l) < 10: 
                        self.flag_l = True
                    elif tickdiff_l > 0:
                        wheel_adj.vel_right = 2* FIXED_SPEED
                    elif tickdiff_l < 0:
                        rospy.loginfo(f"should not be here left {tickdiff_l}")
                        wheel_adj.vel_right = - 2* FIXED_SPEED
                 
                #check if final position is reached for right wheel        
                elif np.abs(self.current_distance[1]) > np.abs(end_distance[1]):
                    self.fine_adjust = True
                    rospy.loginfo(f"{end_distance} , {self.current_distance}")
                    rospy.loginfo(f"current ticks, right reached: {self.current_ticks}")
                    wheel_cmd.v = 0
                    wheel_cmd.omega = 0
                    #tick difference
                    tickdiff_r = np.abs(self.current_ticks[1]) - np.abs(self.current_ticks[0])
                    #check if wheels reached similar distance
                    if np.abs(tickdiff_r) < 10:
                        self.flag_r = True
                    elif tickdiff_r > 0:
                        wheel_adj.vel_left = 2 * FIXED_SPEED
                    elif tickdiff_r < 0:
                        rospy.loginfo(f"should not be here right{tickdiff_r}")
                        wheel_adj.vel_left = - 2 * FIXED_SPEED
                
            return wheel_cmd, wheel_adj, self.fine_adjust
        
        
        ################################
        ### controller using simulated velocity
        ################################
        
        def get_wheel_cmd_vel(self, ticks) -> WheelsCmdStamped:
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
                rospy.loginfo("All done")
                wheel_cmd.vel_left = 0
                wheel_cmd.vel_right = 0
                self.all_commands_excecuted = True
                
            else:
                
                time = self.counter/self.update_rate
                self.current_ticks[0] = ticks[0] - self.init_tick[0]
                self.current_ticks[1] = ticks[1] - self.init_tick[1]
                #rospy.loginfo("current ticks: ", self.current_ticks, " time: ", time)
                    
                # Get the current command
                current_command = self.commands[self.current_command_index]
                command_type = current_command.type  
                direction = current_command.direction  
                distance = current_command.distance

                if command_type == MotionCommand.Type.STRAIGHT:
                    #rospy.loginfo("Straight")
                    end_distance = [distance, distance]
                    dist_tol = 0.01
                    
                    self.goal_distance[0] += FIXED_SPEED/self.update_rate 
                    self.goal_distance[1] += FIXED_SPEED/self.update_rate 
                    self.current_distance[0] = self.dist_per_tick * self.current_ticks[0]
                    self.current_distance[1] = self.dist_per_tick * self.current_ticks[1]
                

                elif command_type == MotionCommand.Type.ROTATE:
                    #rospy.loginfo("Rotate")
                    sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1
                    # Rotate the robot, either clockwise or counterclockwise
                    self.goal_distance[0] += sign * FIXED_SPEED/self.update_rate 
                    self.goal_distance[1] += -sign * FIXED_SPEED/self.update_rate 
                    self.current_distance[0] = self.dist_per_tick * self.current_ticks[0]
                    self.current_distance[1] = self.dist_per_tick * self.current_ticks[1]
                    end_distance = [sign * distance * WHEEL_DISTANCE/2, -sign * distance * (WHEEL_DISTANCE)/2] #distance from rad to m
                    dist_tol = 0.005  
                       
                elif command_type == MotionCommand.Type.CURVE:
                    #rospy.loginfo("Curve")
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
                    
                # rospy.loginfo("current distance: ", round(self.current_distance[0],3) , round(self.current_distance[1],3))
                # rospy.loginfo("Goal distance: ", round(self.goal_distance[0],3) , round(self.goal_distance[1],3))
                # rospy.loginfo("End distance: ", round(end_distance[0],3), round(end_distance[1],3))
                
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
                    #rospy.loginfo("Command done")
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
                

        ################################
        ### controller following predefined trajectory points
        ### -- work in progress --
        ################################
        def get_wheel_cmd_traj(self, ticks, end_coord) -> WheelsCmdStamped:
            """
            Calculate the wheel command based on the current pose and the target trajectory.
            Args:
                cur_pose (Pose): The current pose of the robot.
            Returns:
                WheelsCmdStamped: The command for the robot's wheels.
            """
            
            wheel_cmd = WheelsCmdStamped()
              
            time = self.counter/self.update_rate
            self.current_ticks[0] = ticks[0] - self.init_tick[0]
            self.current_ticks[1] = ticks[1] - self.init_tick[1]
            rospy.loginfo("current ticks: ", self.current_ticks, " time: ", time)
    
            
            max_wheel_speed = FIXED_SPEED  # Maximum wheel speed in rad/s
            wheel_radius = WHEEL_RADIUS
            wheel_base = WHEEL_DISTANCE
        
                
            # Calculate the angle to the next point
            dx = end_coord[0] - self.current_pose[0]
            dy = end_coord[1] - self.current_pose[1]
            target_theta = math.atan2(dy, dx)
            
            # Calculate the angular difference
            delta_theta = target_theta - self.current_pose.theta
            
            # Normalize the delta_theta to the range [-pi, pi]
            delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
            
            # Compute wheel rotations for on-the-spot rotation
            if abs(delta_theta) > 1e-6:  # If there's a significant rotation to perform
                rospy.loginfo("rotation")
                duration = abs(delta_theta) * wheel_base / (2 * max_wheel_speed * wheel_radius)
                
                left_wheel_rotation = delta_theta * wheel_base / (2 * wheel_radius)
                right_wheel_rotation = -left_wheel_rotation

                left_wheel_speed = max_wheel_speed if delta_theta < 0 else -max_wheel_speed
                right_wheel_speed = -left_wheel_speed
                
                kinematic_sequences.append(KinematicSequence(left_wheel_rotation, right_wheel_rotation, left_wheel_speed, right_wheel_speed, duration))

            # Update the robot's orientation after the rotation
            self.current_pose.theta = target_theta
            
            # Compute wheel rotations for straight-line movement
            distance = math.sqrt(dx**2 + dy**2)
            if distance > 1e-6:  # If there's a significant distance to move
                rospy.loginfo("distance")
                duration = distance / (max_wheel_speed * wheel_radius)
            
                left_wheel_rotation = distance / wheel_radius
                right_wheel_rotation = distance / wheel_radius
                
                left_wheel_speed = max_wheel_speed
                right_wheel_speed = max_wheel_speed

                kinematic_sequences.append(KinematicSequence(left_wheel_rotation, right_wheel_rotation, left_wheel_speed, right_wheel_speed, duration))
            
            # Update the robot's position after the translation
            current_pose.update_pose(end_point[0], end_point[1], target_theta)
            
            # Optionally, handle final pose adjustment if there's a small remaining angular offset
            final_theta = final_pose.theta
            delta_theta = final_theta - current_pose.theta
            delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
            
            if abs(delta_theta) > 1e-6:  # Final orientation adjustment
                rospy.loginfo("adjust")
                duration = abs(delta_theta) * wheel_base / (2 * max_wheel_speed * wheel_radius)
                    
                left_wheel_rotation = delta_theta * wheel_base / (2 * wheel_radius)
                right_wheel_rotation = -left_wheel_rotation

                left_wheel_speed = max_wheel_speed if delta_theta < 0 else -max_wheel_speed
                right_wheel_speed = -left_wheel_speed
                kinematic_sequences.append(KinematicSequence(left_wheel_rotation, right_wheel_rotation, left_wheel_speed, right_wheel_speed, duration))
                
            return kinematic_sequences
        
        
        ################################
        ### controller with pose feedback
        ################################
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
        
        ##############################
        ### open loop controller
        ##############################
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
                rospy.loginfo("All done")
                wheel_cmd.vel_left = 0
                wheel_cmd.vel_right = 0
                self.all_commands_excecuted = True
                
            else:
                # Get the current command
                current_command = self.commands[self.current_command_index]
                command_type = current_command.type.value  
                direction = current_command.direction
                distance = current_command.distance
                
                if command_type == MotionCommand.Type.STRAIGHT:
                    rospy.loginfo("Straight")
                    speed = FIXED_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_SPEED
                    wheel_cmd.vel_left, wheel_cmd.vel_right = speed, speed
                    
                    current_distance = time * speed

                elif command_type == MotionCommand.Type.ROTATE:
                    rospy.loginfo("Rotate")
                    # Rotate the robot, either clockwise or counterclockwise
                    angular_speed = FIXED_ROTATION_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_ROTATION_SPEED   # Example speed for rotation
                    wheel_cmd.vel_left, wheel_cmd.vel_right = -angular_speed, angular_speed
                    
                    current_distance = angular_speed * (WHEEL_DISTANCE/2) * time
                    
                    
                elif command_type == MotionCommand.Type.CURVE:
                    rospy.loginfo("Curve")
                    # Move in a curve with a specified radius
                    radius = current_command.radius 
                    sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1
                    wheel_cmd.vel_left = FIXED_SPEED * (1 + sign * (WHEEL_DISTANCE / (2 * radius))) 
                    wheel_cmd.vel_right = FIXED_SPEED * (1 - sign * (WHEEL_DISTANCE / (2 * radius)))
                    
                    avg_vel = (wheel_cmd.vel_right + wheel_cmd.vel_left)/2
                    
                    current_distance = avg_vel * time / radius
                    
                rospy.loginfo("current distance: ", round(current_distance,3),"   Distance: ", round(distance,3))
                
                
                
                if current_distance >= distance:
                    rospy.loginfo("Command done")
                    #reinit
                    current_distance = 0
                    self.current_command_index += 1
                    wheel_cmd.vel_left = 0
                    wheel_cmd.vel_right = 0
                    self.counter = 0
                else:
                    self.counter += 1
                    
            return wheel_cmd
        

        ############################
        ### helper functions
        ############################
        
        
        def controller(self, target_distance):
            error_distance = [0,0]
            error_distance[0] = target_distance[0] - self.current_distance[0] #if (self.goal_distance[0] - self.current_distance[0]) >= 0 else 0 
            error_distance[1] = target_distance[1] - self.current_distance[1] #if (self.goal_distance[0] - self.current_distance[1]) >= 0 else 0 
            
            self.wheel_vel_l = self.kp_l * error_distance[0]
            self.wheel_vel_r = self.kp_r * error_distance[1]
            
            return self.wheel_vel_l, self.wheel_vel_r
        
        
        def move_straight_feedback(self, direction, cur_pose):
            # Move in a straight line, either forward or backward
            speed = FIXED_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_SPEED
            
            #Get current distances
            self.distance_current = np.sqrt((np.abs(cur_pose.position.x) - np.abs(self.initial_pose.position.x))**2 + (np.abs(cur_pose.position.y) - np.abs(self.initial_pose.position.y))**2)
            current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
            rospy.loginfo("Going Straight: current ", self.distance_current, " speed ", speed, ", yaw ", current_yaw)
            
            return speed, speed     
        
        
        def rotate_feedback(self, direction, cur_pose):
            angular_speed = FIXED_ROTATION_SPEED if direction == MotionCommand.Direction.POSITIVE else -FIXED_ROTATION_SPEED   # Example speed for rotation
            
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
            self.distance_current = np.abs(self.initial_yaw - current_yaw) + TOL_ROTATE
            rospy.loginfo("Rotate: current ", self.distance_current, " speed ", angular_speed, ", yaw ", current_yaw )

            return -angular_speed, angular_speed
        
        def move_on_curve_feedback(self, direction, cur_pose, radius):
            sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1 
            
            # get current and inital orientation
            current_yaw = tf.euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])[2]
            self.distance_current = np.abs(self.wraptopi((current_yaw - self.initial_yaw))) + TOL_CURVE
            rospy.loginfo("Making a curve: current ", self.distance_current, " speed ", FIXED_ANGULAR_SPEED)
    
            return FIXED_ANGULAR_SPEED * (1 - sign * (WHEEL_DISTANCE / (2 * radius))), FIXED_ANGULAR_SPEED * (1 + sign * (WHEEL_DISTANCE / (2 * radius)))
        
        def wraptopi(angle):
            #wraps to -pi to pi, also for multiturn
            angle = angle % (2*np.pi)
            if angle > np.pi:
                angle -= 2*np.pi
            else:
                angle += 2*np.pi
            return angle
        