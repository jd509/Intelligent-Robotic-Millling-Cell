#ifndef COORDINATOR
#define COORDINATOR

#include <string>
#include <bits/stdc++.h> 
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>



class Coordinator
{
    public:
    ros::NodeHandle nh_coord;
    std_msgs::String msg_to_gui;


    //Ros Publishers
    ros::Publisher command_rob_1_pub;
    ros::Publisher command_rob_2_pub;
    ros::Publisher send_update_pub;
    
    ros::Subscriber rob_1_sub;
    ros::Subscriber rob_2_sub;
    ros::Subscriber gui_msgs_sub;

    //Constructors and other functions
    Coordinator();

    void initialize_robots();

    void send_update_to_gui(std::string );

    void gui_callback(const std_msgs::String& );

    void rob1_callback(const std_msgs::String& );

    void rob2_callback(const std_msgs::String& );
};


#endif