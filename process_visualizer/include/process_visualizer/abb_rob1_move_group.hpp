#ifndef ABB_ROBOT_1_MOVEGROUP
#define ABB_ROBOT_1_MOVEGROUP

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

class ABB_ROBOT_1
{
private:
    /* data */
public:
    
    // Initializing ROS Parameters

    ros::NodeHandle node_handle_rob1;
    std::string ROBOT_DESCRIPTION = "abb_robot_1/robot_description";
    std::string PLANNING_GROUP = "abb_robot_1/manipulator";
    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle_rob1};
    


    // Initializing MoveGroup Parameters

    moveit::planning_interface::MoveGroupInterfacePtr abb_rob1_movegrp_ptr= std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //const robot_state::JointModelGroup* joint_model_group;
    

    //Functions to perform operations

    ABB_ROBOT_1();
    
    void move_to_configuration(std::vector<double>& joint_angles);
};

ABB_ROBOT_1::ABB_ROBOT_1()
{
    std::cout<<"Yes!";
    ros::WallDuration(1.0).sleep();
}

#endif