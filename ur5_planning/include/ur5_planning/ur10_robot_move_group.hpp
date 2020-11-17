#ifndef UR10_ROBOT_MOVE_GROUP
#define UR10_ROBOT_MOVE_GROUP

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Grasp.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometric_shapes/shape_operations.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Move_Group_Robot_3
{
private:
    /* data */
public:
    
    // Initializing ROS Parameters
    ros::NodeHandle node_handle_rob3;
    std_msgs::String update_msg;
    Eigen::Matrix3d rotation = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix4d world_to_part_T = Eigen::Matrix4d::Zero(4,4);
    // int workpiece_id = 0;
    // int current_wp = 0;

    Eigen::Vector3d workpiece_position = Eigen::Vector3d::Zero();



    std::string PLANNING_GROUP = "manipulator";
    std::string ROBOT_DESCRIPTION = "ur10_robot/robot_description";

    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle_rob3};
    
    moveit::planning_interface::MoveGroupInterfacePtr ur10_robot_group_ptr;

    // Initializing MoveGroup Parameters
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan ur10_robot_cartesian_plan;
    moveit::planning_interface::MoveGroupInterface::Plan ur10_robot_goal_plan;
    

    moveit_msgs::OrientationConstraint goal_pose_constraint;
    
    std::vector<std::string> joint_names;
    std::vector<std::string> link_names;
    std::vector<geometry_msgs::Pose> wp_pos;

    //Publishers and Subscribers
    ros::Subscriber receive_data_from_coord_sub;
    // ros::Subscriber workpiece_pos_sub;
    ros::Publisher send_update_pub;
    ros::Publisher planning_scene_diff_publisher;

    Eigen::Matrix4d hom_T(Eigen::Vector3d t, Eigen::Matrix3d r);

    //Eigen::MatrixXd make_transformation_mat();

    //Functions to perform operations
    Move_Group_Robot_3();
    
    void move_to_configuration(std::vector<double>& joint_angles);

    // void add_robot_table();

    // void add_workpiece_table();

    // void add_workpiece_table_2();

    void send_update(std::string );

    void perform_actions(const std_msgs::String& );

    // void add_collision_obj_to_world(moveit_msgs::CollisionObject object, std::string object_name);

    // void add_workpieces(const geometry_msgs::Pose& );

    // void pick();

    // void place();

    std::string intToString (int a);

    void move_to_pose(geometry_msgs::Pose );

    void perform_milling();

    Eigen::MatrixXd file_read_mat(std::string file_name);

    Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat);
};


#endif