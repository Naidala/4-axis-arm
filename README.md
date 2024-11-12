# Serial Manipulator Kinematics

The serial manipulator is parameterized following the Denavit-Hartenberg convention.

The Jacobian matrix represents the relationship between the joint velocities and the end-effector twist vector. A CLIK (Closed-Loop Inverse Kinematics) method brings the end-effector to the desired start position.


### MATLAB Directory

The `MATLAB` directory contains the following elements:

- **Scripts and Functions**: MATLAB scripts for implementing kinematic calculations and control algorithms, primarily for simulating CLIK. 
- **Simulink Models**: The directory includes a Simulink model (`simulink_clik.slx`) that represents the kinematic chain and control loop of the robotic arm. 

### ROS Directory

The `ROS` directory is designed for real-time control implementations and includes nodes written in C++ and Python to handle CLIK and path control using ROS. Here’s an overview of its contents:

- **`gcode_interpreter.py` Node**: This Python node reads a G-code file and extracts a sequence of target coordinates for the end-effector to follow, converting these into ROS messages.

  - *Publisher*: Publishes `Position_EE` messages containing desired end-effector coordinates on the `coordinates` topic.
  - *Execution of G-code*: Loops through the G-code lines and sends each target position in sequence, with CLIK used for fast repositioning and path control for smooth motion along the path points. 
  
- **`joint_publisher.cpp` Node**: This C++ node calculates joint velocities and positions to drive the arm's end-effector toward the desired coordinates. It subscribes to the `coordinates` topic for receiving target end-effector positions and publishes joint states to the `joint_states` topic.

  - *Callback Function*: Receives messages with desired positions and control mode, storing them in global variables for easy access.
  - *Main Loop*: Computes the inverse kinematics and publishes joint commands. Uses the desired end-effector position to calculate joint angles and velocities, based on the CLIK algorithm.
