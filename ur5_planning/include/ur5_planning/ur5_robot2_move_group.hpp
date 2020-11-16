#ifndef UR5_ROBOT2_MOVE_GROUP
#define UR5_ROBOT2_MOVE_GROUP

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shape_operations.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

class Move_Group_Robot_2
{
private:
    /* data */
public:
    
    // Initializing ROS Parameters
    ros::NodeHandle node_handle_rob2;
    ros::Subscriber receive_data_from_coord_sub;
    ros::Publisher send_update_pub;
    ros::Publisher planning_scene_diff_publisher;

    std_msgs::String update_msg;
    std::vector<std::vector<double>> bin_1_pos = {{1.53, -0.43, 0.56},{1.53, -0.6, 0.56}};
    std::vector<std::vector<double>> bin_2_pos = {{1.53, -0.85, 0.56},{1.53, -0.97, 0.56}};
    std::vector<double> milling_table_pos;

    std::string PLANNING_GROUP = "manipulator";
    std::string ROBOT_DESCRIPTION = "ur5_robot2/robot_description";

    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle_rob2};
    
    moveit::planning_interface::MoveGroupInterfacePtr ur5_robot2_group_ptr;

    // Initializing MoveGroup Parameters
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan ur5_robot2_cartesian_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5_robot2_goal_plan;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    moveit_msgs::OrientationConstraint goal_pose_constraint;
    
    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;
    int bin1_wp = 0;
    int bin2_wp = 0;


    //Functions to perform operations
    Move_Group_Robot_2();
    
    void move_to_configuration(std::vector<double>& joint_angles);

    void add_robot_table();

    void send_update(std::string );

    std::string intToString (int a);

    void perform_actions(const std_msgs::String& );

    void pick();

    void place_bin1();

    void place_bin2();

    void place_fr_milling();

    void move_to_pose(geometry_msgs::Pose target_p);

};

#endif