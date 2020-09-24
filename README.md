# A_star- simulation of turtlebot in ROS

The scripts (inside the turtlebot package can be run seperately as the publisher node is included 
They can be executed by :- python3 <name_of_the_script>

In this project phase 3 is implemented onto turtlebot3 using ros.
The entire package is included in the zip file.
The name of node is astar and it publishes topic on /cmd_vel.
The nodes are constantly published according to the step size.
Ros publish 15~20 values in one sec hence the linear velocities are scaled accordingly.
The action space taken for algorithm is scaled (multiplied) to factor of 10 and is calculated for exact dimensions of turtlebot

To visulise the simulation gazebo is used and the a seperate map called map.xacro is used.

The RPMs for the turtlebot taken are 50 and 100

FOR VIDEO 1 :- SCRIPT phase4_ros.py works best
FOR VIDEO 2 :- SCRIPT phase4_Ros.py works best
although phase4_ros.py is the main code and is executable for all valid nodes, phase4_Ros.py works faster for video2.

please export turtlebot burger before launching the file.
