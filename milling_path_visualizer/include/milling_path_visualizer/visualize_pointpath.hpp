#ifndef VISUALIZE_POINTPATH
#define VISULAIZE_POINTPATH

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <chrono>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Waypoints
{
public:

    ros::NodeHandle n_millpath;
    ros::Subscriber getpoints_sub;
    ros::Publisher displaypoints_pub;
    

    geometry_msgs::Point clicked_point;
    visualization_msgs::Marker pointlist;
    visualization_msgs::MarkerArray list;
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    int marker_id = 1;

    std::vector<std::vector<float>> milling_waypoints;

    Waypoints();

    void getwaypoints(const geometry_msgs::PointStamped& );

    void setwaypointslist(const float &, const float&, const float&);

    void file_rw(std::vector<std::vector<float>> waypoints);

};

#endif