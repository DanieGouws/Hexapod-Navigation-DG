# Hexapod-Navigation-DG
Repo for Masters Project on control and navigation algorithms for an existing 18 servo hexapod robot
This project is based on the work done in the master's theses of Cristopher Ross, Sheldon Erasmus and Philip Lotriet on development of the hexapod robot.
This robot is an 18 servo mechanical system driven by a Jetson Nano and Teensy. It has an RGBD camera to observe the terrain and adapt to it.
In particular this repo will focus on duplicating their control methods and then extending them to more complex uneven terrain and similar applications of interest. The system will also be upgraded to run on modern ROS2 middleware.

# ToDo
- Add code for the Simscape simulation. Add setup instructions 
- Add the hexapod code for both the teensy and the Jetson
- Add instructions referencing hardware description and setup.
- Add description of software setup for hexapod.

# Contents of the Repo:
The repo is a clone of a ROS2 Jazzy workspace directory. This allows it to contain multiple packages which can be run with the use of colcon build and then $source install/setup.bash. For convenient access, the arduino code for the Teensy is also kept in the top level of the repo.
- src: 
  -  my_hexapod_interfaces
      - Format for the required msg files for publish and subscribe.  
  -  sheldon_ros2_port2
      - Actual ros and python code responsible for high level motion control. 
