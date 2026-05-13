# Simulator for Spot robot using Gazebo and ROS 2 Humble.

this project is a simulation environment for exploration algorithms with the boston dynamics robot spot. It uses Gazebo Ignition Fortress as the simulator and ROS 2 Humble for communication and control.



Structure of the project:

- Algorithms: Contains the exploration algorithms implemented in Python.
- Core: Adapter to interface between ROS2 with Gazebo and real robot sdk code
- spot: all script related to the real robot and all the spot sdk calls
- spot_ros: ros implementations of the spot sdk calls