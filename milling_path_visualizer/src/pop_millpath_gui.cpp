#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include "milling_path_visualizer/pop_millpath_gui.hpp"

PopMillingGUI::PopMillingGUI()
{
    load_milling_gui = n_popgui.subscribe("/open_millpath_gui", 1000, &PopMillingGUI::OpenMillingGUI, this);
    milling_gui_clean = n_popgui.advertise<std_msgs::String>("/clean_milling_gui_before_relaunch", 1000);
    load_wp = n_popgui.advertise<std_msgs::String>("/visualize_wp", 1000);
}

void PopMillingGUI::OpenMillingGUI(const std_msgs::String& gui_msg)
{
    //ROS_INFO_STREAM(gui_msg.data);
	int stat = system("rosrun rviz rviz -d `rospack find milling_path_visualizer`/rviz/milling_path_gui.rviz");
	std_msgs::String msg;
	msg.data = "cleanGUI";		
    milling_gui_clean.publish(msg);
    load_wp.publish(gui_msg);
}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "Pop_MillPath_GUI");
    
    PopMillingGUI pop_gui;

    while(ros::ok())
    {
    	ros::Rate loop_rate(1000);

  		ros::spinOnce();
  		loop_rate.sleep();
    }  
return 0;
}

