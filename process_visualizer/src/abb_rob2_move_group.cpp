#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb_robot2");
  ros::NodeHandle node_handle_rob2;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string PLANNING_GROUP = "abb_robot_2/manipulator";
  std::string robot_description = "abb_robot_2/robot_description";

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_ptr = std::make_shared<robot_model_loader::RobotModelLoader>(robot_description);
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader_ptr->getModel();
  moveit::core::RobotStatePtr kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
  urdf::ModelInterfaceSharedPtr Robot_Model = robot_model_loader_ptr->getURDF();

  
  moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, robot_description, node_handle_rob2};
  moveit::planning_interface::MoveGroupInterfacePtr abb_rob2_movegrp_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup* joint_model_group;
  joint_model_group = abb_rob2_movegrp_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  std::cout<<"///////////////////////////////////      Model Loaded     ////////////////////////////////// \n";
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", abb_rob2_movegrp_ptr->getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s",abb_rob2_movegrp_ptr->getEndEffectorLink().c_str());

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(abb_rob2_movegrp_ptr->getJointModelGroupNames().begin(), abb_rob2_movegrp_ptr->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  std::cout<<"////////////////////////////////    Planning started    /////////////////////////////// \n";

  moveit::core::RobotStatePtr current_state = abb_rob2_movegrp_ptr->getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = -0.5;  // radians
  abb_rob2_movegrp_ptr->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (abb_rob2_movegrp_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  abb_rob2_movegrp_ptr->execute(my_plan.trajectory_);
  std::cout<<my_plan.trajectory_<<std::endl;
  
  //move_group.stop();
  std::cout<<"////////////////////////////////  Planning Completed    ////////////////////////////////// \n";


  ros::waitForShutdown();
  return 0;
}