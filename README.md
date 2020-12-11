#Monte Carlo Localization - Terrain Environment (Husky)
This github contemplates the implementation Monte Carlo Localization using a particle filter.

#Prerquisites
- Ubuntu 16.04 - https://ubuntu.com/tutorials/install-ubuntu-desktop-1604#1-overview
- ROS Kinetic - http://wiki.ros.org/kinetic/Installation/Ubuntu

#Steps to run the code
- Download this repository
- Insert it in your catkin_ws/src
- Insert the following commands:
```
roscd
catkin build
```

Having your package built, open several terminal windows and run:
- On the first terminal:
```
roscore
```
On the second terminal, introduce your user and one map name (playpen OR fastfood), and run:
```
roslaunch husky_gazebo husky_empty_world.launch world_name:=/home/<user>/catkin_ws/src/mclocalization/Maps/<map_name>.world
```
On the third terminal:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```
On the fourth terminal. introduce your user and one map name (playpen OR fastfood), and run:
```
rosrun map_server map_server /home/<user/catkin_ws/src/mclocalization/Maps/<map_name>.yaml
```
On the fifth terminal:
```
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom map
```
On the sixth terminal:
```
rosrun rviz rviz 
```
On the seventh terminal, with a defined number of particles and rate:
```
rosrun mclocalization mcl.py <number of particles> <rate>
```

Open rviz, select add (bottom left), and add the Map and PoseArray topics.

Enjoy!
