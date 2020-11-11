# Intelligent Robotic Milling Cell

This repository contains code to execute a deep-learning based robotic milling cell in ROS framework using Python and C++.

## List of Required Dependencies

The following dependencies need to be installed before running the project.
- [ROS-melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) - framework for running the project.
- [Gazebo](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros) - for simulation of the robot motions.
- [MoveIt](https://moveit.ros.org/install/) - Motion Planning of Robots.

## Installation
 
Clone the repository in a folder and rename it to src. For now, please clone the master branch of the repo using:

```bash
git clone -b master https://github.com/jd509/Intelligent-Robotic-Millling-Cell.git
```

Then, type the following on a terminal:

```bash
cd YOUR_FOLDER && catkin_make -j7
```
This will make the project. 

## Usage

For simulation and planning purpose:

Open a new terminal and paste the following code:
```bash
cd YOUR_FOLDER
source devel/setup.bash
roslaunch process_visualizer start_simulation.launch
```
This will start the gazebo with the needed controllers. The initial PID gain have not been set and hence may produce warnings. 

The gazebo window should display two robots as follows:

![Gazebo World](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/images/gazebo_initial_image.png)


Now, open another terminal alongside the gazebo and type the following:
```bash
cd YOUR_FOLDER
source devel/setup.bash
roslaunch ur5_planning ur5_planning.launch
```
This will start the MoveIt's Motion Planning Framework. The RViz window can then be used to plan and execute robot motions.

## Known Issues
If the gazebo specific controllers are not installed on the system, MoveIt will not be able to execute motions. You can install all the ros-controllers by typing:
```bash
sudo apt-get install ros*controller*
```
## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
