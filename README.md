# A-star Path Planning simulation of TurtleBot using ROS

This project is the execution of Autonomous path planning on the differential drive turtlebot. The algorithm used is a search algorithm called A-star which is an optimal discrete planning. There are several random obstacles and the bot plans the most optimal way to the goal position. For this simulation non-holonomic constrainst are given to the robot which means that it cannot move in y-direction independently.

## Differential drive equations

```
x_dot = r*(ul + ur)*cos(theta)/2
y_dot = r*(ul + ur)*sin(theta)/2
theta_dot = r*(ur - ul)/l
```
## Demonstrations

![A-star gif](media/A-star.gif)
![Astar gif](media/Astar.gif)

## Prerequisites

ROS kinetic or ROS melodic is required to launch the simulation.After cloning the repository the turtlebot folder can be used as it is as a ROS workspace. 

## Astar Execution

![Flowchart jpg](media/Flowchart.jpg)


