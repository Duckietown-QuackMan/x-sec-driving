from enums import MotionCommand, DistanceType, Mission, Command
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
import matplotlib.pyplot as plt
import rospy
import math
import numpy as np
    
#init params
WHEEL_DISTANCE = 0.102 #102
WHEEL_RADIUS = 0.035
FIXED_SPEED = 0.2 #m/s   
BOT_CORRECTION = 0.7  #gimpy
ADJUSTMENT_TOL = 10

debug_print = False

class XsecNavigator:
    
    def __init__(self, move_straight_params, move_right_params, move_left_params):
        """ 
        initialize missions of XsecNavigator path, moving left, right and straight.
        
        Args:
            move_straight_params: list[str] = path parameters for moving straight
            move_right_params: list[str] = path parameters for moving right
            move_left_params: list[str] = path parameters for moving left
        Output:
            --
        """
        
        # mission straight 
        self.move_straight = Mission(
            name="move_straight",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_straight_params["distance"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
            ]
        )
        #
        self.move_right = Mission(
            name="move_right",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_right_params["distance_before"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
                Command(type=MotionCommand.Type.ROTATE, 
                        direction=MotionCommand.Direction.POSITIVE, 
                        distance=np.pi/2,
                        ),
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_right_params["distance_after"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
            ]
        )
        

        self.move_left = Mission(
            name="move_left",
            commands=[
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_left_params["distance_before"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
                Command(type=MotionCommand.Type.ROTATE, 
                        direction=MotionCommand.Direction.NEGATIVE, 
                        distance=np.pi/2,
                        ),
                Command(type=MotionCommand.Type.STRAIGHT, 
                        distance=move_left_params["distance_after"],
                        direction=MotionCommand.Direction.POSITIVE, 
                        ),
            ]
        )
         
    class Move():
        def __init__(self, commands=[], init_ticks=(), resolution=135):
            """
            Initialize the Move class with initial pose and commands.
            Args:
                commands (list): A list of motion commands for the robot to execute.
            """
            
            self.commands = commands            # List of motion commands.
            self.current_command_index = 0      # Track which command is being executed.
            self.init_tick = init_ticks         #[left, right]
            self.all_commands_excecuted = False 
            self.dist_per_tick = 2*np.pi/resolution * WHEEL_RADIUS
            self.goal_distance = [0, 0]         #[left, right]
            self.current_distance = [0, 0]      #[left, right]
            self.current_ticks = [0, 0]         #[left, right]
            self.radius_wheel = [0, 0]          #[left, right]
            self.flag_done = False              #set to true if fine adjustment is done
            self.flag_l = False                 #set to true if left wheel is fine adjusted
            self.flag_r = False                 #set to true if right wheel is finde adjusted
            self.fine_adjust = False            #fine adjustement 
        
        def get_wheel_cmd_ticks(self, ticks):
            """
            Calculate the wheel command and the adjustment commands based on the current ticks.
            Args:
                ticks: List[int,int] = The current ticks of the wheel encoder.
            Returns:
                wheel_cmd : Twist2DStamped = wheel cmd of velocity and omega during main navigation
                wheel_adj : WheelsCmdStamped = wheel adjustement cmd of right and left wheel speed during adjustment phase.
                
            """
            wheel_cmd = Twist2DStamped()
            wheel_adj = WheelsCmdStamped()

            # Check if commands have been executed, stop the robot
            if self.current_command_index >= len(self.commands):
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
                #get current ticks relative to the start of the command
                self.current_ticks[0] = ticks[0] - self.init_tick[0]
                self.current_ticks[1] = ticks[1] - self.init_tick[1]
                #get current distance for the current command
                self.current_distance[0] = self.dist_per_tick * self.current_ticks[0]
                self.current_distance[1] = self.dist_per_tick * self.current_ticks[1] 
                # Get the current command parameters
                current_command = self.commands[self.current_command_index]
                command_type = current_command.type  
                direction = current_command.direction  
                distance = current_command.distance
                
                ###
                # command driving 
                ###
                # # encode command parameters
                if command_type == MotionCommand.Type.STRAIGHT:
                    #set straight parmas
                    end_distance = [distance, distance]
                    wheel_cmd.v = FIXED_SPEED
                    wheel_cmd.omega = 0
                    if debug_print:
                        rospy.loginfo(f"Straight")
                        rospy.loginfo(f"{end_distance}, {self.current_distance}")
    
                elif command_type == MotionCommand.Type.ROTATE:
                    if debug_print:
                        rospy.loginfo(f"Rotate")
                        rospy.loginfo(f"{end_distance} , {self.current_distance}")
                        rospy.loginfo(f"current ticks: {self.current_ticks}")
                    # Rotate the robot, either clockwise or counterclockwise
                    sign = 1 * BOT_CORRECTION if direction == MotionCommand.Direction.POSITIVE else -1 
                    end_distance = [sign * distance * WHEEL_DISTANCE/2, -sign * distance * WHEEL_DISTANCE/2] #distance from rad to m
                    wheel_cmd.v = 0
                    wheel_cmd.omega = -sign * FIXED_ANGULAR_SPEED                        
                       
                elif command_type == MotionCommand.Type.CURVE:
                    if debug_print:
                        rospy.loginfo("Curve")
                    # Move in a curve with a specified radius (TODO backwords curve)
                    sign = 1 if direction == MotionCommand.Direction.POSITIVE else -1
                    radius = current_command.radius
                    #calculate end distance of curve tile 
                    self.radius_wheel[0] = radius + sign * WHEEL_DISTANCE / 2
                    self.radius_wheel[1] = radius - sign * WHEEL_DISTANCE / 2
                    end_distance = [distance * self.radius_wheel[0], distance * self.radius_wheel[1]] #distance from rad to m
                    wheel_cmd.v = FIXED_SPEED
                    wheel_cmd.omega = FIXED_SPEED/radius 
                ###    
                # fineadjustment
                ###
                
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
                    wheel_cmd.v = 0
                    wheel_cmd.omega = 0
                    #tick difference left wheel
                    tickdiff_l = np.abs(self.current_ticks[0]) - np.abs(self.current_ticks[1])
                    #check if wheels reached similar distance
                    if np.abs(tickdiff_l) < ADJUSTMENT_TOL: 
                        self.flag_l = True
                    #check if right wheel is behind or ahead
                    elif tickdiff_l > 0:
                        wheel_adj.vel_right = 2*FIXED_SPEED
                    elif tickdiff_l < 0:
                        wheel_adj.vel_right = - 2*FIXED_SPEED
                 
                #check if final position is reached for right wheel        
                elif np.abs(self.current_distance[1]) > np.abs(end_distance[1]):
                    self.fine_adjust = True
                    wheel_cmd.v = 0
                    wheel_cmd.omega = 0
                    #tick difference right wheel
                    tickdiff_r = np.abs(self.current_ticks[1]) - np.abs(self.current_ticks[0])
                    #check if wheels reached similar distance
                    if np.abs(tickdiff_r) < ADJUSTMENT_TOL:
                        self.flag_r = True
                    #check if left wheel is behind or in front
                    elif tickdiff_r > 0:
                        wheel_adj.vel_left = 2*FIXED_SPEED
                    elif tickdiff_r < 0:
                        wheel_adj.vel_left = - 2*FIXED_SPEED
                
            return wheel_cmd, wheel_adj, self.fine_adjust
        
       