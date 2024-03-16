# ROS package
This is an ROS package that allows the control of a six-axis Easy Robots robot using an attached force and torque measurement device.
## Description
It consists of two nodes written in C++ and a robot control program written in Python. The task of the first node is to communicate with the force and torque measurement device, and to publish the received data on the ROS channel. The second node keeps track of the robot's current position and the force applied to the arm tip. Based on these, it calculates new positions and publishes them on the ROS channel. A program running on the robot receives the sentepositions and moves the manipulator tip to them.

## Example
How the application was tested is shown below. When a force was applied, the robot was observed to move towards it.
![Image](https://github.com/MateuszKochanski/mk_inz/blob/master/images/testy_.png)

## Used

[ROS](https://www.ros.org/),  [Ubuntu](https://ubuntu.com/download), [EasyRobots](https://easyrobots.pl/)
