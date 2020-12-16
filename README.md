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
- Insert the following commands, while replacing ```<user>``` by your machine user:

```
roscd
mv /home/<user>/catkin_ws/src/Sistemas-Autonomos-20_21-main/ /home/<user>/catkin_ws/src/mclocalization
catkin build
sudo apt-get install ros-kinetic-teleop-twist-keyboard
sudo apt-get install python-pip
sudo pip install numpy scipy
```

Having your package built, open several terminal windows and run:
- On the first terminal:
```
roscore
```
On the second terminal, in ```<user>```, substitute by your user and in ```<map_name>``` substitute by ```playpen```, ```fastfoodcar``` OR ```new_city```, and run:
```
roslaunch husky_gazebo husky_empty_world.launch world_name:=/home/<user>/catkin_ws/src/mclocalization/Maps/<map_name>.world
```
On the third terminal:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```
On the fourth terminal,  in ```<user>```, substitute by your user and in ```<map_name>``` substitute by ```playpen```,```fastfood2``` OR ```new_map_city``` depending on the world that you selected on the second terminal, and run:
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
On the seventh terminal, in ```<number_of_particules>```, substitute by a number (ex: 100, 200, 500, 1000, 2000, ...) and in ```<rate>``` substitute by a rate in Hz (0.5, 1, 2, ...). For the average teste in Playpen, we reccomend 500 particales with a rate of 2 Hz:
```
rosrun mclocalization mcl.py <number_of_particles> <rate>
```

Open rviz, select option "Add" (bottom left), and add the Map and PoseArray topics. If desired, the LaserScan topic can be added too.
In order to move the husky, select the third terminal and follow the instructions on the terminal window.


In order to check the results, we reccomend checking the Particles positions in Rviz and the Husky position in the Gazebo program and comparing with each other.

If you want to see the robot location directly in Rviz, a topic of the Husky "true" location may be added by adding the topic "InteractiveMarkers", however, sometimes this suffers a deviation, and doesn't represent its true position (in case of kidnapping, it mantains the last position which is wrong), so we don't recommend it.


Enjoy!










## Guide to fixing errors

- If you have built your catkin previously with catkin make, you need to run the command:
```catkin clean -y```
And then run:
```catkin build```


- If you are getting an error saying that the mclocation package was not found, check if you executed the commands presented in the beggining of this guide. If you did, replace ```<user>``` with your machine user and run:
```source /home/<user>/catkin_ws/devel/setup.bash```


- If you are still getting the same error, replace ```<user>``` with your machine user and run the following commands:
```cd /home/<user>/catkin_ws/src/mclocalization/src```
```chmod +x mcl.py```

In ```<number_of_particules>```, substitute by a number (ex: 100, 200, 500, 1000, 2000, ...) and in ```<rate>``` substitute by a rate in Hz (0.5, 1, 2, ...).  Run the following command:
```python mcl.py <number_of_particules> <rate>```
