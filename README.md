# Monte Carlo Localization - Terrain Environment (Husky)
This github contemplates the implementation of Monte Carlo Localization using a particle filter.


![ezgif com-video-to-gif](https://user-images.githubusercontent.com/74827101/101970614-50ad7900-3c23-11eb-80f7-70c2bae2ef1e.gif)


## Prerequisites
- Ubuntu 16.04 - https://ubuntu.com/tutorials/install-ubuntu-desktop-1604#1-overview
- ROS Kinetic - http://wiki.ros.org/kinetic/Installation/Ubuntu
- Familiarize yourself with the Catkin tools.

## Steps to run the code
- Download this repository and extract it.
- Insert it in your catkin_ws/src.
- Insert the following commands:
```
roscd
catkin build
sudo apt-get install ros-kinetic-teleop-twist-keyboard
```

Having your package built, open several terminal windows and run:
- On the first terminal:
```
roscore
```
On the second terminal, in <user>, substitute by your user and in <map_name> substitute by playpen OR fastfood, and run:
```
roslaunch husky_gazebo husky_empty_world.launch world_name:=/home/<user>/catkin_ws/src/mclocalization/Maps/<map_name>.world
```
On the third terminal:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```
On the fourth terminal,  in <user>, substitute by your user and in <map_name> substitute by playpen OR fastfood, and run:
```
rosrun map_server map_server /home/<user>/catkin_ws/src/mclocalization/Maps/<map_name>.yaml
```
On the fifth terminal:
```
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom map
```
On the sixth terminal:
```
rosrun rviz rviz 
```
On the seventh terminal, in <number_of_particules>, substitute by a number (ex: 50, 75, 100, 150, 200, ...) and in <rate> substitute by a rate in Hz (0.5, 1, 2, ...). For the average teste in Playpen, we reccomend 100 particales with a rate of 1 Hz (do not use numbers of particules much higher than the ones stated as it requires a lot of resources from your computer):
```
rosrun mclocalization mcl.py <number_of_particles> <rate>
```

Open rviz, select option "Add" (bottom left), and add the Map and PoseArray topics. If desired, the LaserScan topic can be added too.
In order to move the husky, select the third terminal and follow the instructions on the terminal window.
When the Husky gets localized, a message on the seventh terminal will pop up saying "Husky Localized!"


In order to check the results, we reccomend checking the Particles positions in Rviz and the Husky position in the Gazebo program and comparing with each other.

If you want to see the robot location directly in Rviz, a topic of the Husky "true" location may be added by adding the topic "InteractiveMarkers", however, sometimes this suffers a deviation, and doesn't represent its true position, so we don't recommend it.


Enjoy!
