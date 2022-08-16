# Husky Line Coverage
In this project, we combine [Line Coverage library](https://github.com/UNCCharlotte-CS-Robotics/LineCoverage-library) and [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) to achieve autonomous and accurate coverage of a road network.

[PDF Poster](https://github.com/ninhnt285/husky_line_coverage/blob/main/Poster.pdf) | [Youtube Video](https://www.youtube.com/watch?v=7oildxYRykk)

![3d Map](https://github.com/ninhnt285/husky_line_coverage/blob/main/3D%20Map.png)

# How to run package

## Option 1: Run on robot
1. Run LeGO-LOAM
```
roslaunch lego-loam run.launch
```
2. Run husky_line_coverage
```
roslaunch husky_line_coverage run.launch on_robot:=true route_dir:=atsp2
```
3. Start autonomous robot
```
rostopic pub /is_pause std_msgs/Bool "data: false"
```


## Option 2: Run on simulator (gazebo)
1. Run gazebo
```
roslaunch husky_gazebo husky_empty_world.launch
```
2. Run husky_line_coverage
```
roslaunch husky_line_coverage run.launch is_simulation:=true route_dir:=atsp2
```
3. Start autonomous robot
```
rostopic pub /is_pause std_msgs/Bool "data: false"
```


## Option 3: Run bag files
1. Run husky_line_coverage
```
roslaunch husky_line_coverage run.launch is_bag:=true route_dir:=atsp2
```
2. Run rosbag
