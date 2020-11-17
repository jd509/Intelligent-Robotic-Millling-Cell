#ifndef LOAD_WP_MILLING
#define LOAD_WP_MILLING

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <chrono>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <string>

class Milling_Workpiece_Visualizer
{
public:

    ros::NodeHandle nh;
    ros::Subscriber load_wp_sub;
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    
    std::string file_name;
    geometry_msgs::Pose P;

    bool flag = false;

    Milling_Workpiece_Visualizer();

    void getwpinfo(const std_msgs::String& );
    void publishwp();
};

#endif