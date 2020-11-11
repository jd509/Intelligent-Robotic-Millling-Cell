#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb_robot3");
  ros::NodeHandle node_handle_rob3;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher display_publisher =
      node_handle_rob3.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  std::string PLANNING_GROUP = "abb_robot_3/manipulator";
  std::string robot_description = "abb_robot_3/robot_description";

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("abb_robot_3/base");
  visual_tools.loadRobotStatePub("abb_robot_3/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // clear all old markers
  visual_tools.trigger();

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_ptr = std::make_shared<robot_model_loader::RobotModelLoader>(robot_description);
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader_ptr->getModel();
  moveit::core::RobotStatePtr kinematic_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
  urdf::ModelInterfaceSharedPtr Robot_Model = robot_model_loader_ptr->getURDF();

  
  moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, robot_description, node_handle_rob3};
  moveit::planning_interface::MoveGroupInterfacePtr abb_rob3_movegrp_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);

  //abb_rob3_movegrp_ptr->startStateMonitor('abb_robot_3/joint_states');

  ros::Duration(10.0).sleep();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group;
  joint_model_group = abb_rob3_movegrp_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  std::cout<<"///////////////////////////////////      Model Loaded     ////////////////////////////////// \n";

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", abb_rob3_movegrp_ptr->getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s", abb_rob3_movegrp_ptr->getEndEffectorLink().c_str());

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(abb_rob3_movegrp_ptr->getJointModelGroupNames().begin(), abb_rob3_movegrp_ptr->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));





  std::cout<<"////////////////////////////////    Planning started    /////////////////////////////// \n";

  moveit::core::RobotStatePtr current_state = abb_rob3_movegrp_ptr->getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = -0.5;  // radians
  abb_rob3_movegrp_ptr->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (abb_rob3_movegrp_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  abb_rob3_movegrp_ptr->execute(my_plan.trajectory_);
  //abb_rob3_movegrp_ptr->stop();
  //std::cout<<my_plan.trajectory_<<std::endl;
  std::cout<<my_plan.trajectory_.joint_trajectory.points[0]<<std::endl;
  std::cout<<my_plan.trajectory_.joint_trajectory.points.back()<<std::endl;


  moveit::core::RobotStatePtr current_state_2 = abb_rob3_movegrp_ptr->getCurrentState();
  std::vector<double> joint_group_positions_2;
  joint_model_group = current_state_2->getJointModelGroup(PLANNING_GROUP);
  current_state_2->copyJointGroupPositions(joint_model_group, joint_group_positions_2);

  for(size_t i = 0; i<joint_group_positions_2.size(); i++)
  {
    std::cout<<"Joint "<<i+1<<": "<<joint_group_positions_2[i]<<std::endl;
  }

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = abb_rob3_movegrp_ptr->getPlanningFrame();
  collision_object.id = "box1";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);

  abb_rob3_movegrp_ptr->setStartState(*abb_rob3_movegrp_ptr->getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.4;
  another_pose.position.y = -0.4;
  another_pose.position.z = 0.9;
  abb_rob3_movegrp_ptr->setPoseTarget(another_pose);

  success = (abb_rob3_movegrp_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  abb_rob3_movegrp_ptr->attachObject(collision_object.id);
  visual_tools.trigger();

  abb_rob3_movegrp_ptr->detachObject(collision_object.id);
  visual_tools.trigger();
  
  abb_rob3_movegrp_ptr->move();
  // abb_rob3_movegrp_ptr->stop();

  
//   robot_state::RobotState start_state(*abb_rob3_movegrp_ptr->getCurrentState());
//   std::vector<double> joint_group_positions_new = {-1.0, 0, 0, 0, 0, 0};

  

//   //joint_group_positions_new[0] = -0.5;  // radians
//   abb_rob3_movegrp_ptr->setStartState(start_state);
//   abb_rob3_movegrp_ptr->setJointValueTarget(joint_group_positions_new);

//   //moveit::planning_interface::MoveGroupInterface::Plan my_plan_new;
//   success = (abb_rob3_movegrp_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   abb_rob3_movegrp_ptr->execute(my_plan.trajectory_);
// //  std::cout<<my_plan.trajectory_<<std::endl;
//   std::cout<<my_plan.trajectory_.joint_trajectory.points[0]<<std::endl;
//   std::cout<<my_plan.trajectory_.joint_trajectory.points.back()<<std::endl;


  //move_group.stop();
  std::cout<<"////////////////////////////////  Planning Completed    ////////////////////////////////// \n";


  ros::waitForShutdown();
  return 0;
}