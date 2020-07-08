# hector_quadrotor_tutorial
Hector quadrotor with modified files

Installed required packages
 ```
sudo apt-get install ros-kinetic-moveit ros-kinetic-velodyne-gazebo-plugins ros-kinetic-geographic-info ros-kinetic-geographic-msgs ros-kinetic-ros-control ros-kinetic-hardware-interface ros-kinetic-controller-interface ros-kinetic-gazebo-ros-control ros-kinetic-joy ros-kinetic-teleop-twist-keyboard
 ```

## Hector Quadrotor
To launch the quadrotor in the gazebo cafe
 ```
roslaunch hector_quadrotor_demo cafe.launch
 ```
To launch the quadrotor in the simulated soybean farm
 ```
roslaunch hector_quadrotor_demo soy_flight_gazebo.launch
 ```
## Hector Quadrotor with Moveit
To launch the quadrotor in the gazebo cafe with the moveit frame
 ```
roslaunch hector_moveit_gazebo cafe.launch
 ```
To launch the exploration or navigation node
 ```
roslaunch hector_moveit_exploration explore.launch
 ```
 ```
roslaunch hector_moveit_navigation navigate.launch
 ```
## References
<a href="https://github.com/tahsinkose/hector-moveit">Hector Quadrotor with MoveIt! Motion Planning Framework by Tahsincan Köse</a>

<a href="https://github.com/wilselby/ROS_quadrotor_simulator">ROS Quadrotor Simulator by Wil Selby</a>

<a href="https://github.com/AlessioTonioni/Autonomous-Flight-ROS">Autonomous Flight by Alessio Tonioni</a>
