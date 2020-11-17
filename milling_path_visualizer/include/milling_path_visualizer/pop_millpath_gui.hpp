#ifndef POP_MILLPATH_GUI
#define POP_MILLPATH_GUI

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <chrono>
#include <iostream>

class PopMillingGUI
{
public:

    ros::NodeHandle n_popgui;
    ros::Subscriber load_milling_gui;
    ros::Publisher milling_gui_clean;
    ros::Publisher load_wp;

    PopMillingGUI();

    void OpenMillingGUI(const std_msgs::String& );

};

#endif