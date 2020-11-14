#ifndef UR5_ROBOT1_MOVE_GROUP
#define UR5_ROBOT1_MOVE_GROUP

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
#include <geometry_msgs/Pose.h>

class Move_Group_Robot_1
{
private:
    /* data */
public:
    
    // Initializing ROS Parameters
    ros::NodeHandle node_handle_rob1;
    std_msgs::String update_msg;
    int workpiece_id = 0;


    std::string PLANNING_GROUP = "manipulator";
    std::string ROBOT_DESCRIPTION = "ur5_robot1/robot_description";

    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle_rob1};
    
    moveit::planning_interface::MoveGroupInterfacePtr ur5_robot1_group_ptr;

    // Initializing MoveGroup Parameters
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan ur5_robot1_cartesian_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur5_robot1_goal_plan;
    

    moveit_msgs::OrientationConstraint goal_pose_constraint;
    
    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;

    //Publishers and Subscribers
    ros::Subscriber receive_data_from_coord_sub;
    ros::Subscriber workpiece_pos_sub;
    ros::Publisher send_update_pub;
    ros::Publisher planning_scene_diff_publisher;


    //Functions to perform operations
    Move_Group_Robot_1();
    
    void move_to_configuration(std::vector<double>& joint_angles);

    void add_robot_table();

    void add_workpiece_table();

    void send_update(std::string );

    void perform_actions(const std_msgs::String& );

    void add_collision_obj_to_world(moveit_msgs::CollisionObject object, std::string object_name);

    void add_workpieces(const geometry_msgs::Pose& );

    std::string intToString (int a);
};


#endif