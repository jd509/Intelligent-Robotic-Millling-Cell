#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <chrono>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include "milling_path_visualizer/visualize_pointpath.hpp"
#include <ros/package.h>


Waypoints::Waypoints()
{
    getpoints_sub = n_millpath.subscribe("/clicked_point", 1000, &Waypoints::getwaypoints, this);
    displaypoints_pub = n_millpath.advertise<visualization_msgs::Marker>("/publish_points_to_gui", 1000);
}

void Waypoints::getwaypoints(const geometry_msgs::PointStamped &P)
{
    Waypoints::setwaypointslist(P.point.x, P.point.y, P.point.z);
}

void Waypoints::setwaypointslist(const float& x, const float& y, const float& z)
{
    std::vector<float> temp;
    clicked_point.x = x;
    clicked_point.y = y;
    clicked_point.z = z + 0.1;
    float w = 1.0, x_o = 0.0, y_o = 0.0, z_o = 0.0;
    float x_p, y_p, z_p;
    x_p = clicked_point.x;
    y_p = clicked_point.y;
    z_p = clicked_point.z;  

    pointlist.header.frame_id = "ur10_robot_tf/world";
    pointlist.type = visualization_msgs::Marker::SPHERE_LIST;
    pointlist.action = visualization_msgs::Marker::ADD;
    pointlist.lifetime = ros::Duration();
    pointlist.id = marker_id;
    pointlist.scale.x = 0.01;
    pointlist.scale.y = 0.01;
    pointlist.scale.z = 0.01;
    pointlist.color.a = 1.0;
    pointlist.color.r = 0.0;
    pointlist.color.g = 0.0;
    pointlist.color.b = 1.0; 
    pointlist.pose.orientation.w = 1.0;

    pointlist.points.push_back(clicked_point);

    temp.push_back(clicked_point.x);
    temp.push_back(clicked_point.y);
    temp.push_back(clicked_point.z);
    milling_waypoints.push_back(temp);
    for(auto i:milling_waypoints)
    {
        std::cout<<i[0]<<" "<<i[1]<<" "<<i[2];
    }
    // milling_waypoints.push_back(w);
    // milling_waypoints.push_back(x_o);
    // milling_waypoints.push_back(y_o);
    // milling_waypoints.push_back(z_o);
    std::string file_name = "/home/jaineel/Desktop/AME_547_Project/Jaineel/Jaineel_WS_New/src/milling_path_visualizer/data/waypoints.csv";
    std::ofstream out_file;
    std::cout<<"writing to file \n";
    out_file.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    for (long i=0;i<milling_waypoints.size();++i)
    {
        for (long j=0;j<milling_waypoints[i].size();++j)
        {
            if (j!=milling_waypoints[i].size()-1)
            {
                out_file << milling_waypoints[i][j] << ",";    
            }
            else
            {
                out_file << milling_waypoints[i][j] << std::endl;       
            }
        }
    }
    out_file.close();

//    file_rw(milling_waypoints);
    //list.markers.push_back(pointlist);
    displaypoints_pub.publish(pointlist);
    marker_id++;
}

void Waypoints::file_rw(std::vector<std::vector<float>> waypoints)
{
    std::string file_name = ros::package::getPath("milling_path_visualizer")+"data/waypoints.csv";
    std::ofstream out_file;
    std::cout<<"writing to file \n";
    out_file.open(file_name.c_str(),std::ios::out);
    for (long i=0;i<waypoints.size();++i)
    {
        for (long j=0;j<waypoints[i].size();++j)
        {
            if (j!=waypoints[i].size()-1)
            {
                out_file << waypoints[i][j] << ",";    
            }
            else
            {
                out_file << waypoints[i][j] << std::endl;       
            }
        }
    }
    out_file.close();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Publish_Milling_Lines");
    ROS_INFO_STREAM("Visualizing points..");
    Waypoints pts;
    while(ros::ok())
    {
    	ros::Rate loop_rate(1000);
  		ros::spinOnce();

  		loop_rate.sleep();
    }  
    return 0;
}



