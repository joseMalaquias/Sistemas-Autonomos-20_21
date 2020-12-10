# Sistemas-Autonomos-20_21
Open 4 different terminals

On the first terminal run:
```
roscore
```
On the second terminal, open the map file directory and run:
```
rosrun map_server map_server mapName.yaml
```
On the the third terminal, open the code.py directory and run: 
```
python code.py <number_of_particles_desired>
```
On the fourth terminal run:
```
rosrun rviz rviz
```
To visualize the map and particles in Rviz:
- Select add (bottom left of the screen) and add topic Map
- Select add (bottom left of the screen) and add topic PoseArray




