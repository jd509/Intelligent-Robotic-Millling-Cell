#include <ros/ros.h>
#include <std_msgs/String.h>
#include <chrono>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <string>
#include "milling_path_visualizer/load_milling_workpiece.hpp"

Milling_Workpiece_Visualizer::Milling_Workpiece_Visualizer()
{
    load_wp_sub = nh.subscribe("/visualize_wp", 1000, &Milling_Workpiece_Visualizer::getwpinfo, this);
    file_name = "package://process_visualizer/resources/workpiece.stl";
    P.position.x = 0;
    P.position.y = 0;
    P.position.z = 0;

    P.orientation.x = 0;
    P.orientation.y = 0;
    P.orientation.z = 0;
    P.orientation.w = 1;
}

void Milling_Workpiece_Visualizer::getwpinfo(const std_msgs::String& wp_name)
{
    file_name = "package://process_visualizer/resources/workpiece.stl";
    // publishwp();

}

void Milling_Workpiece_Visualizer::publishwp()
{
    file_name = "package://process_visualizer/resources/workpiece.stl";
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("ur10_robot_tf/world","/publish_wp"));
	visual_tools_->publishMesh(P, file_name, rviz_visual_tools::GREY, 1.0);
	visual_tools_->trigger();
}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "Millpath_wp_Viz");
    Milling_Workpiece_Visualizer wpObj;

    while(ros::ok())
    {
    	ros::Rate loop_rate(1000);
        wpObj.publishwp();
  		ros::spinOnce();
  		loop_rate.sleep();

    }  
return 0;
}
