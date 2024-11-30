#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np
from tf.transformations import euler_from_quaternion
from include.enums import MotionCommand, Shape


class GtVis:
    def __init__(self, shape: str, initial_pose: PoseStamped, commands: list):
        """
        Initialize the GtVis class.

        Args:
            shape (str): The shape to visualize (e.g., square, circle, figure-eight).
            initial_pose (PoseStamped): The initial pose of the robot.
            commands (list): A list of motion commands to visualize.
        """
        if not (
            str(Shape.SQUARE) in shape
            or str(Shape.CIRCLE) in shape
            or str(Shape.EIGHT) in shape
        ):
            txt = f"Invalid shape provided '{shape}"
            rospy.logerr(txt)
            raise ValueError(txt)

        self.shape = shape
        self.initial_2d_pose = self.get_pose(initial_pose)

        self.commands = commands  # Commands from the path planner
        self.frame_id = "world"

    def get_pose(self, initial_pose: PoseStamped) -> list:
        """
        Convert the initial pose from a PoseStamped to a 2D pose [x, y, theta].

        Args:
            initial_pose (PoseStamped): The initial pose of the robot.

        Returns:
            list: A list containing the x, y, and theta values of the initial pose.
        """
        initial_pose_x = initial_pose.position.x
        initial_pose_y = initial_pose.position.y
        orientation_quat = [
            initial_pose.orientation.x,
            initial_pose.orientation.y,
            initial_pose.orientation.z,
            initial_pose.orientation.w,
        ]
        initial_2d_pose_theta = euler_from_quaternion(orientation_quat)[2]
        initial_2d_pose = [initial_pose_x, initial_pose_y, initial_2d_pose_theta]

        return initial_2d_pose

    def get_rotation_matrix(self, theta: float, x: float, y: float) -> np.ndarray:
        """
        Create a rotation matrix for a rotation of theta around the point (x, y).

        Args:
            theta (float): The angle of rotation in radians.
            x (float): The x-coordinate of the point around which to rotate.
            y (float): The y-coordinate of the point around which to rotate.

        Returns:
            np.ndarray: The rotation matrix.
        """
        # Rotation matrix for a rotation of theta around the point (x, y)
        R_theta = [
            [np.cos(theta), -np.sin(theta), x - x * np.cos(theta) + y * np.sin(theta)],
            [np.sin(theta), np.cos(theta), y - x * np.sin(theta) - y * np.cos(theta)],
            [0, 0, 1],
        ]
        return R_theta

    def create_base_marker(self) -> Marker:
        """
        Create a base marker with common properties.

        Returns:
            Marker: A ROS Marker message with basic properties set.
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        marker.scale.x = 0.01

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        return marker

    def create_square_marker(self) -> Marker:
        """
        Create a marker representing a square.

        Returns:
            Marker: A ROS Marker message representing the square.
        """
        marker = self.create_base_marker()
        marker.ns = str(Shape.SQUARE)
        marker.id = 0

        # Get the side length of the square from the first command
        if len(self.commands) != 7:
            a = 1.0
            rospy.logerr(
                "Wrong number of commands provided. Defaulting side length of square to 1.0"
            )
        elif (
            self.commands[0][0] == MotionCommand.Type.STRAIGHT
            and self.commands[1][0] == MotionCommand.Type.ROTATE
        ):
            a = self.commands[0][-1]
            dir = self.commands[1][1]
        else:
            a = 1.0
            dir = MotionCommand.Direction.NEGATIVE
            rospy.logerr(
                "Wrong command provided. Defaulting side length of square to 1.0"
            )

        # Define the square vertices (4 corners + first point to close the square)
        x, y, theta = (
            self.initial_2d_pose[0],
            self.initial_2d_pose[1],
            self.initial_2d_pose[2],
        )
        if dir == MotionCommand.Direction.NEGATIVE:
            square_corner_matrix = [
                [x, y, 0.0],  # First point
                [x + a, y, 0.0],  # Second point
                [x + a, y - a, 0.0],  # Third point
                [x, y - a, 0.0],  # Fourth point
                [x, y, 0.0],
            ]  # Close the square
        else:
            square_corner_matrix = [
                [x, y, 0.0],  # First point
                [x + a, y, 0.0],  # Second point
                [x + a, y + a, 0.0],  # Third point
                [x, y + a, 0.0],  # Fourth point
                [x, y, 0.0],
            ]  # Close the square

        # Rotate the points by the initial pose theta
        R_theta = self.get_rotation_matrix(theta, x, y)
        square_points = []
        for point in square_corner_matrix:
            homogeneous_point = np.array([point[0], point[1], 1])
            [x, y, _] = R_theta @ homogeneous_point
            # Append the points to the list
            square_points.append(Point(x, y, 0.0))

        marker.points = square_points
        return marker

    def create_circle_marker(self) -> Marker:
        """
        Create a marker representing a circle.

        Returns:
            Marker: A ROS Marker message representing the circle.
        """
        marker = self.create_base_marker()
        marker.ns = str(Shape.CIRCLE)
        marker.id = 1

        # Get the radius of the circle from the command
        if len(self.commands) != 1:
            radius = 1.0
            dir = MotionCommand.Direction.NEGATIVE
            rospy.logerr(
                "Wrong number of commands provided. Defaulting radius of circle to 1.0"
            )
        elif self.commands[0][0] == MotionCommand.Type.CURVE:
            radius = self.commands[0][-1]
            dir = self.commands[0][1]
        else:
            radius = 1.0
            dir = MotionCommand.Direction.NEGATIVE
            rospy.logerr("Wrong command provided. Defaulting radius of circle to 1.0")

        # Define the circle points
        circle_points = []
        num_points_circle = 100  # Number of points to draw the circle
        for i in range(num_points_circle + 1):
            angle = 2 * np.pi * i / num_points_circle
            # Calculate the point coordinates (withouth the initial orientation)
            if dir == MotionCommand.Direction.NEGATIVE:
                x = self.initial_2d_pose[0] + radius * np.sin(angle)
                y = self.initial_2d_pose[1] + radius * (np.cos(angle) - 1)
            else:
                x = self.initial_2d_pose[0] + radius * np.sin(angle)
                y = self.initial_2d_pose[1] + radius * (1 - np.cos(angle))

            # Rotate the points by the initial pose theta
            R_theta = self.get_rotation_matrix(
                self.initial_2d_pose[2],
                self.initial_2d_pose[0],
                self.initial_2d_pose[1],
            )
            homogeneous_point = np.array([x, y, 1])
            [x, y, _] = R_theta @ homogeneous_point
            # Append the point to the list
            circle_points.append(Point(x, y, 0.0))
        marker.points = circle_points

        return marker

    def get_curve_points(self, p_start: list, radius: float, alpha: float, direction: MotionCommand.Direction) -> list:
        """
        Calculate the points for a curve segment in a figure-eight shape.

        Args:
            p_start (list): The starting point of the curve.
            radius (float): The radius of the curve.
            alpha (float): The angle of the curve segment.
            direction (MotionCommand.Direction): The direction of the curve (positive or negative).

        Returns:
            list: A list of points representing the curve segment.
        """
        points_matrix = []
        num_points_circle = 100
        for i in range(num_points_circle):
            angle = (np.pi + alpha) * i / num_points_circle
            # Calculate the point coordinates (without the initial orientation)
            if direction == MotionCommand.Direction.POSITIVE:
                x = p_start[0] + radius * np.sin(angle)
                y = p_start[1] + radius * (1 - np.cos(angle))
            elif direction == MotionCommand.Direction.NEGATIVE:
                x = (
                    p_start[0]
                    - radius * np.sin(angle) * np.cos(alpha)
                    - radius * (1 - np.cos(angle)) * np.sin(alpha)
                )
                y = (
                    p_start[1]
                    - radius * np.sin(angle) * np.sin(alpha)
                    + radius * (1 - np.cos(angle)) * np.cos(alpha)
                )
            # Append the point to the matrix
            points_matrix.append([x, y, 0.0])
        return points_matrix

    def create_figure_eight_marker(self) -> Marker:
        """
        Create a marker representing a figure-eight.

        Returns:
            Marker: A ROS Marker message representing the figure-eight.
        """
        marker = self.create_base_marker()
        marker.ns = str(Shape.EIGHT)
        marker.id = 2

        # Get the radius and alpha of the figure eight from the command
        if len(self.commands) != 5:
            radius = 1.0
            alpha = np.pi / 2
            rospy.logerr(
                "Wrong number of commands provided. Defaulting radius to 1.0, alpha to pi/2"
            )
        elif (
            self.commands[0][0] == MotionCommand.Type.STRAIGHT
            and self.commands[1][0] == MotionCommand.Type.CURVE
        ):
            radius = self.commands[1][3]
            alpha = self.commands[1][2] - np.pi
        else:
            radius = 1.0
            alpha = np.pi / 2
            rospy.logerr(
                "Wrong command provided. Defaulting radius to 1.0, alpha to pi/2"
            )

        # Define the figure eight points
        figure_eight_points_matrix = []
        # Rotation matrix of initial pose theta
        R_theta = self.get_rotation_matrix(
            self.initial_2d_pose[2], self.initial_2d_pose[0], self.initial_2d_pose[1]
        )
        dist_straight = radius / np.tan(alpha / 2)
        # define start/end points for straight lines
        edge_points = [
            [self.initial_2d_pose[0], self.initial_2d_pose[1], 0],
            [self.initial_2d_pose[0] + dist_straight, self.initial_2d_pose[1], 0],
            [
                self.initial_2d_pose[0]
                + dist_straight
                + radius * np.sin(np.pi + alpha),
                self.initial_2d_pose[1] + radius * (1 - np.cos(np.pi + alpha)),
                0,
            ],
            [
                self.initial_2d_pose[0]
                - (dist_straight + radius * np.sin(np.pi + alpha)),
                self.initial_2d_pose[1] - (radius * (1 - np.cos(np.pi + alpha))),
                0,
            ],
            [self.initial_2d_pose[0] - dist_straight, self.initial_2d_pose[1], 0],
            [self.initial_2d_pose[0], self.initial_2d_pose[1], 0],
        ]

        for i in range(len(self.commands)):
            if self.commands[i][0] == MotionCommand.Type.STRAIGHT:
                cur_matrix = [edge_points[i], edge_points[i + 1]]

            elif self.commands[i][0] == MotionCommand.Type.CURVE:
                p_start = edge_points[i]
                cur_matrix = self.get_curve_points(
                    p_start, radius, alpha, self.commands[i][1]
                )
            figure_eight_points_matrix += cur_matrix

        # Rotate the points by the initial pose theta
        figure_eight_points = []
        for point in figure_eight_points_matrix:
            homogeneous_point = np.array([point[0], point[1], 1])
            [x, y, _] = R_theta @ homogeneous_point
            # Append the points to the list
            figure_eight_points.append(Point(x, y, 0.0))

        marker.points = figure_eight_points
        return marker

    # create marker according to shape
    def get_gt_path(self) -> Marker:
        """
        Create a marker according to the specified shape.

        Returns:
            Marker: A ROS Marker message representing the ground truth path for the specified shape.
        """
        if str(Shape.SQUARE) in self.shape:
            return self.create_square_marker()
        elif str(Shape.CIRCLE) in self.shape:
            return self.create_circle_marker()
        elif str(Shape.EIGHT) in self.shape:
            return self.create_figure_eight_marker()
        else:
            raise ValueError(f"Invalid shape provided '{self.shape}")