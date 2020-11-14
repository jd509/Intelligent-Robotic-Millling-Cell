#ifndef WORKPIECE_HANDLER
#define WORKPIECE_HANDLER

#include <gazebo_msgs/SetModelState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <urdf/model.h>
#include <gazebo_msgs/SpawnModel.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shape_operations.h>
#include <std_msgs/String.h>


class Workpiece_Object{

public:

    ros::NodeHandle nh_workpiece;
    int num_of_workpieces_rob1 = 0;
    int current_wp_rob1 = 0;
    int current_wp_rob2 = 0;

    //Publishers and Subscribers
    ros::Publisher gazebo_model_state_pub;
    ros::Subscriber joint_states_robot1_sub;
    ros::Subscriber joint_states_robot2_sub;
    ros::Subscriber initial_workpiece_pos_sub;

    ros::ServiceClient spawnClient;

    //robot model loaders
    robot_model::RobotModelPtr kinematic_model_ur5_1;
    robot_state::RobotStatePtr kinematic_state_ur5_1;


    //Constructor and other functions
    Workpiece_Object();

    std::string intToString(int a);

    void load_workpiece_in_gazebo(const geometry_msgs::Pose& p);

    void rob1_jointstates_callback(const sensor_msgs::JointState &joint_states_current);

    void rob2_jointstates_callback(const sensor_msgs::JointState &joint_states_current);

};





#endif