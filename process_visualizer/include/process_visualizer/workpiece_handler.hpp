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

#include <tf/transform_listener.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/DeleteModel.h>



class Workpiece_Object{

public:

    ros::NodeHandle nh_workpiece;
    int num_of_workpieces_rob1 = 0;
    int current_wp_rob1 = 0;
    int current_wp_rob2 = 0;
    int current_wp_rob3 = 1;

    int i = 0;
    std::vector<std::vector<double>> bin_1_pos = {{1.53, -0.43, 0.56},{1.53, -0.6, 0.56}, {1.53, -0.85, 0.56},{1.53, -0.97, 0.56}};
    std::vector<std::vector<double>> bin_2_pos = {{1.53, -0.85, 0.56},{1.53, -0.97, 0.56}};


    //Publishers and Subscribers
    ros::Publisher gazebo_model_state_pub;
    ros::Subscriber joint_states_robot1_sub;
    ros::Subscriber joint_states_robot2_sub;
    ros::Subscriber initial_workpiece_pos_sub;
    ros::Subscriber attached_to_rob_1;
    ros::Subscriber wp_complete_sub;

    ros::ServiceClient spawnClient;
    ros::ServiceClient deleteModelClient; 

    //robot model loaders
    robot_model::RobotModelPtr kinematic_model_ur5_1;
    robot_state::RobotStatePtr kinematic_state_ur5_1;
    const robot_state::JointModelGroup *joint_model_group;

    tf::TransformListener listener;
    gazebo_msgs::LinkStates state;


    //Constructor and other functions
    Workpiece_Object();

    std::string intToString(int a);

    void load_workpiece_in_gazebo(const geometry_msgs::Pose& p);

    void rob1_jointstates_callback(const gazebo_msgs::LinkStates& state);

    void rob2_jointstates_callback(const sensor_msgs::JointState &joint_states_current);

    void set_rob1_flag(const std_msgs::String & );

    int getIndex(std::vector<std::string> v, std::string K);

    void remove(std::string model_name);

    void spawn_model(std::string model_name, geometry_msgs::Pose p);

    void milling_completion(const std_msgs::String & );

};





#endif