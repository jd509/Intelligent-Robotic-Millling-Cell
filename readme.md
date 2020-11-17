# Intelligent Robotic Milling Cell

This repository contains code to execute a deep-learning based robotic milling cell in ROS framework using Python and C++.

## List of Required Dependencies

The following dependencies need to be installed before running the project.
- [ROS-melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) - framework for running the project.
- [Gazebo](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros) - for simulation of the robot motions.
- [MoveIt](https://moveit.ros.org/install/) - Motion Planning of Robots.
- [Keras](https://keras.io/)-API to process deep learning dataset.
- [TensorFlow](https://www.tensorflow.org/)-API to train Convolutional Neural Networks
- [Sci-kit Learn](https://scikit-learn.org/stable/)-Machine Learning module in Python

## Installation
 
Clone the repository in a folder 'Intelligent_Robotic_Milling_Cell' and rename the cloned repository to src. For now, please clone the master branch of the repo using:

```bash
git clone -b master https://github.com/jd509/Intelligent-Robotic-Millling-Cell.git
```

Then, type the following on a terminal:

```bash
cd Intelligent_Robotic_Milling_Cell && catkin_make -j7
```
This will make the project. 

## Usage

For simulation and planning purpose:

Open a new terminal and paste the following code:
```bash
cd Intelligent_Robotic_Milling_Cell
source devel/setup.bash
roslaunch process_visualizer start_simulation.launch
```
This will start the gazebo with the needed controllers. The initial PID gain have not been set and hence may produce warnings. 

The gazebo window should display the robotic cell as follows:

![Gazebo World](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/images/initial_setup.png)


Now, open another terminal alongside the gazebo and type the following:
```bash
cd Intelligent_Robotic_Milling_Cell
source devel/setup.bash
roslaunch ur5_planning ur5_planning.launch
```
This will start the MoveIt's Motion Planning Framework. The RViz window can then be used to plan and execute robot motions.

Finally, start the User Interface for the cell in the next step as:
```bash
cd Intelligent_Robotic_Milling_Cell
source devel/setup.bash
roslaunch user_interface user_interface.launch
```
## Simulation Video

The simulation video is embedded in the thumbnail below. Please follow the youtube link by clicking on the image.

[![Alt text](https://img.youtube.com/vi/IsruGf38qPI/0.jpg)](https://www.youtube.com/watch?v=IsruGf38qPI)

## Modules
- [Process Visualizer](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/tree/master/process_visualizer)
  
  
  The process visualizer package includes the launch file to load objects in the gazebo world. 
  - ROS node list:
    - [Workpiece Handler](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/process_visualizer/src/workpiece_handler.cpp) : This node is used to handle the workpiece spawning in Gazebo. 
  - [URDF files](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/tree/master/process_visualizer/urdf) : This folder contains all the urdf files needed to spawn objects into the world. 

- [Convolutional Neural Network](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/tree/master/deep_learning_model)

    This package includes the service files, datasets and the pretrained model to classify images using a 6-layered network.

- [Milling Path Generator](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/tree/master/milling_path_visualizer)

    This package includes the nodes to draw millling paths on the workpiece.
  - ROS node list:
    - [Visualizing Milling Path](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/milling_path_visualizer/src/visualize_pointpath.cpp) : This node is used to draw milling paths using points on the workpiece.
    - [Visualizing Workpiece for Milling Path Generation](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/milling_path_visualizer/src/load_milling_workpiece.cpp) : This node is used to visualize the workpiece in RViz.

    ![Milling Path Generator](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/images/milling_path_draw.png)

- [User Interface](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/tree/master/user_interface)

    This package includes the nodes to launch user interface for commanding the robots.
  - ROS node list:
    - [User Interface](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/user_interface/src/user_interface/user_interface_control.py) : This node is used to define button connections for controlling the robot cell.
  - [Qt Design File](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/user_interface/resource/user_interface_gui.ui)

    ![User Interface](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/images/user_interface.png)


- [MoveIt Planning](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/tree/master/ur5_planning)

    This package includes the launch files and nodes to start the moveit planning for each robot.
  - ROS node list:
    - [UR5 Robots](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/ur5_planning/src/ur5_robot1_move_group.cpp) : This node controls the motion planning for UR5 robots.
  - [UR10 Robot](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/ur5_planning/src/ur10_robot_move_group.cpp) : his node controls the motion planning for UR10 Milling robot.

- [Coordinator](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/tree/master/coordinator) : This package controls the information flow in the system.
  
    ![System Rqt Graph](https://github.com/jd509/Intelligent-Robotic-Millling-Cell/blob/master/images/rosgraph.png)
  
  

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
