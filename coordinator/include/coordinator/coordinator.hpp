#ifndef COORDINATOR
#define COORDINATOR

#include <string>
#include <bits/stdc++.h> 
#include <iostream>
#include <ctime>
#include <vector>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "deep_learning_model/CNN.h"



class Coordinator
{
    public:
    ros::NodeHandle nh_coord;
    std_msgs::String msg_to_gui;


    //Ros Publishers
    ros::Publisher command_rob_1_pub;
    ros::Publisher command_rob_2_pub;
    ros::Publisher send_update_pub;
    ros::Publisher coord_to_gazebo_pub;
    ros::Publisher rob_1_attachment_pub;
    
    ros::Subscriber rob_1_sub;
    ros::Subscriber rob_2_sub;
    ros::Subscriber gui_msgs_sub;
    ros::Subscriber num_of_wp_sub;

    ros::ServiceClient deep_learning_service;

    std::vector<std::vector<float>>wp_position = {{1.36,0.6,0.525}, {1.36,0.75,0.525}, {1.36,0.9,0.525},{1.53,0.6,0.525}, {1.53,0.75,0.525},{1.53,0.9,0.525},{1.7,0.6,0.525},{1.7,0.75,0.525},{1.7,0.9,0.525}};
    
    //Constructors and other functions
    Coordinator();

    void initialize_robots();

    void send_update_to_gui(std::string );

    void gui_callback(const std_msgs::String& );

    void rob1_callback(const std_msgs::String& );

    void rob2_callback(const std_msgs::String& );

    void load_workpieces(const std_msgs::Int16& );
};


#endif