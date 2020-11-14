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


class Workpiece_Object{

public:

    ros::NodeHandle nh_workpiece;
    int num_of_workpieces = 0;
    
    //Publishers and Subscribers
    ros::Publisher gazebo_model_state_pub;
    ros::Subscriber joint_states_robot1_sub;
    ros::Subscriber joint_states_robot2_sub;
    ros::Subscriber initial_workpiece_pos_sub;

    ros::ServiceClient spawnClient;

    //Constructor and other functions
    Workpiece_Object();

    std::string intToString(int a);

    void load_workpiece_in_gazebo(const geometry_msgs::Pose& p);

    void rob1_jointstates_callback(const sensor_msgs::JointState &joint_states_current);

    void rob2_jointstates_callback(const sensor_msgs::JointState &joint_states_current);

};





#endif