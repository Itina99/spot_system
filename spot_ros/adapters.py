""" adapt ros to robot interface"""
from Core.robot_interface import (LocalGridProvider, StateProvider, MovementProvider, VisualizerProvider, RecordingProvider)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class ROSLocalGridProvider(LocalGridProvider):
    def get_obstacle_distance_grid(self):
        # implementazione specifica per ROS
        pass
class ROSVisualizerProvider(VisualizerProvider):

class ROSRecordingProvider(RecordingProvider):

class ROSStateProvider(StateProvider):

class ROSMovementProvider(MovementProvider):

