#include "process_visualizer/abb_rob1_move_group.hpp"


void ABB_ROBOT_1::move_to_configuration(std::vector<double>& joint_angles)
{
  // moveit::core::RobotStatePtr current_state =abb_rob1_movegrp_ptr->getCurrentState();
  
  // moveit::planning_interface::MoveGroupInterface::Plan abb_rob1_plan;
  std::vector<double> target_joint_angles = {1.5, 1.2, 0.5, 3.12, 1.23, 4.5};
  std::vector<double> target_joint_angles_2 = {2.1, 1.5, 0.75, 3.12, 1.98, 4.5};

  // std::vector<double> current_joint_positons;
  // current_state->copyJointGroupPositions(joint_model_group, current_joint_positons);
  abb_rob1_movegrp_ptr->startStateMonitor(1.0);

  abb_rob1_movegrp_ptr->setJointValueTarget(target_joint_angles);
  moveit::planning_interface::MoveGroupInterface::Plan abb_rob1_plan;

  bool success = (abb_rob1_movegrp_ptr->plan(abb_rob1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
// abb_rob1_plan.trajectory_
  abb_rob1_movegrp_ptr->execute(abb_rob1_plan);
  std::cout<<abb_rob1_plan.trajectory_.joint_trajectory.points[0]<<std::endl;
  std::cout<<abb_rob1_plan.trajectory_.joint_trajectory.points.back()<<std::endl;
  
  // robot_state::RobotState start_state(abb_rob1_movegrp_ptr->getCurrentState())
  //joint_model_group = abb_rob1_movegrp_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // moveit::core::RobotStatePtr current_state = abb_rob1_movegrp_ptr->getCurrentState();
  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // for (auto i : joint_group_positions)
  // {
  //   std::cout<<"Value is: "<<i<<std::endl;
  // }
  // abb_rob1_movegrp_ptr->setStartState(target_joint_angles);
  // abb_rob1_movegrp_ptr->startStateMonitor(1.0);


  abb_rob1_movegrp_ptr->setJointValueTarget(target_joint_angles_2);
//  moveit::planning_interface::MoveGroupInterface::Plan abb_rob1_plan;

  
  success = (abb_rob1_movegrp_ptr->plan(abb_rob1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
// abb_rob1_plan.trajectory_
  abb_rob1_movegrp_ptr->execute(abb_rob1_plan);
  std::cout<<abb_rob1_plan.trajectory_.joint_trajectory.points[0]<<std::endl;
  std::cout<<abb_rob1_plan.trajectory_.joint_trajectory.points.back()<<std::endl;

  // move_group.stop();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb_robot1");
  ABB_ROBOT_1 rob1_obj;
  ros::AsyncSpinner spinner(1);
  spinner.start();
        


  // std::cout<<"///////////////////////////////////      Model Loaded     ////////////////////////////////// \n";

  // ROS_INFO_NAMED("tutorial", "Planning frame: %s", abb_rob1_movegrp_ptr->getPlanningFrame().c_str());

  // ROS_INFO_NAMED("tutorial", "End effector link: %s", abb_rob1_movegrp_ptr->getEndEffectorLink().c_str());

  // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  // std::copy(abb_rob1_movegrp_ptr->getJointModelGroupNames().begin(), abb_rob1_movegrp_ptr->getJointModelGroupNames().end(),
  //           std::ostream_iterator<std::string>(std::cout, ", "));

  std::cout<<"////////////////////////////////    Planning started    /////////////////////////////// \n";

  std::cout<<"Executing joint angles [1.5, 1.2, 0.5, 3.12, 1.23, 4.5] \n";
  
  std::vector<double> target_joint_angles = {1.5, 1.2, 0.5, 3.12, 1.23, 4.5};

  rob1_obj.move_to_configuration(target_joint_angles);

  // rob1_obj.abb_rob1_movegrp_ptr->setStartStateToCurrentState();

  // std::cout<<"Executing joint angles [2.1, 1.5, 0.75, 3.12, 1.98, 4.5] \n";

  // std::vector<double> target_joint_angles_2 = {2.1, 1.5, 0.75, 3.12, 1.98, 4.5};

  // rob1_obj.move_to_configuration(target_joint_angles_2);


  std::cout<<"////////////////////////////////  Planning Completed    ////////////////////////////////// \n";


  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  // moveit_msgs::OrientationConstraint ocm;
  // ocm.link_name = "panda_link7";
  // ocm.header.frame_id = "panda_link0";
  // ocm.orientation.w = 1.0;
  // ocm.absolute_x_axis_tolerance = 0.1;
  // ocm.absolute_y_axis_tolerance = 0.1;
  // ocm.absolute_z_axis_tolerance = 0.1;
  // ocm.weight = 1.0;

  // // Now, set it as the path constraint for the group.
  // moveit_msgs::Constraints test_constraints;
  // test_constraints.orientation_constraints.push_back(ocm);
  // move_group.setPathConstraints(test_constraints);

  // // We will reuse the old goal that we had and plan to it.
  // // Note that this will only work if the current state already
  // // satisfies the path constraints. So, we need to set the start
  // // state to a new pose.
  // robot_state::RobotState start_state(*move_group.getCurrentState());
  // geometry_msgs::Pose start_pose2;
  // start_pose2.orientation.w = 1.0;
  // start_pose2.position.x = 0.55;
  // start_pose2.position.y = -0.05;
  // start_pose2.position.z = 0.8;
  // start_state.setFromIK(joint_model_group, start_pose2);
  // move_group.setStartState(start_state);

  // // Now we will plan to the earlier pose target from the new
  // // start state that we have just created.
  // move_group.setPoseTarget(target_pose1);

  // // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  // move_group.setPlanningTime(10.0);

  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishAxisLabeled(start_pose2, "start");
  // visual_tools.publishAxisLabeled(target_pose1, "goal");
  // visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("next step");

  // // When done with the path constraint be sure to clear it.
  // move_group.clearPathConstraints();

  // // Cartesian Paths
  // // ^^^^^^^^^^^^^^^
  // // You can plan a Cartesian path directly by specifying a list of waypoints
  // // for the end-effector to go through. Note that we are starting
  // // from the new start state above.  The initial pose (start state) does not
  // // need to be added to the waypoint list but adding it can help with visualizations
  // std::vector<geometry_msgs::Pose> waypoints;
  // waypoints.push_back(start_pose2);

  // geometry_msgs::Pose target_pose3 = start_pose2;

  // target_pose3.position.z -= 0.2;
  // waypoints.push_back(target_pose3);  // down

  // target_pose3.position.y -= 0.2;
  // waypoints.push_back(target_pose3);  // right

  // target_pose3.position.z += 0.2;
  // target_pose3.position.y += 0.2;
  // target_pose3.position.x -= 0.2;
  // waypoints.push_back(target_pose3);  // up and left

  // // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  // move_group.setMaxVelocityScalingFactor(0.1);

  // // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // // which is why we will specify 0.01 as the max step in Cartesian
  // // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // // Warning - disabling the jump threshold while operating real hardware can cause
  // // large unpredictable motions of redundant joints and could be a safety issue
  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  // for (std::size_t i = 0; i < waypoints.size(); ++i)
  //   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // // Adding/Removing Objects and Attaching/Detaching Objects
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // Define a collision object ROS message.
  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = move_group.getPlanningFrame();

  // // The id of the object is used to identify it.
  // collision_object.id = "box1";

  // // Define a box to add to the world.
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 0.4;
  // primitive.dimensions[1] = 0.1;
  // primitive.dimensions[2] = 0.4;

  // // Define a pose for the box (specified relative to frame_id)
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.4;
  // box_pose.position.y = -0.2;
  // box_pose.position.z = 1.0;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.push_back(collision_object);

  // // Now, let's add the collision object into the world
  // ROS_INFO_NAMED("tutorial", "Add an object into the world");
  // planning_scene_interface.addCollisionObjects(collision_objects);

  // // Show text in RViz of status
  // visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // // Wait for MoveGroup to recieve and process the collision object message
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // // Now when we plan a trajectory it will avoid the obstacle
  // move_group.setStartState(*move_group.getCurrentState());
  // geometry_msgs::Pose another_pose;
  // another_pose.orientation.w = 1.0;
  // another_pose.position.x = 0.4;
  // another_pose.position.y = -0.4;
  // another_pose.position.z = 0.9;
  // move_group.setPoseTarget(another_pose);

  // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // // Visualize the plan in RViz
  // visual_tools.deleteAllMarkers();
  // visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("next step");

  // // Now, let's attach the collision object to the robot.
  // ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  // move_group.attachObject(collision_object.id);

  // // Show text in RViz of status
  // visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // /* Wait for MoveGroup to recieve and process the attached collision object message */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
  //                     "robot");

  // // Now, let's detach the collision object from the robot.
  // ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  // move_group.detachObject(collision_object.id);

  // // Show text in RViz of status
  // visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // /* Wait for MoveGroup to recieve and process the attached collision object message */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
  //                     "robot");

  // // Now, let's remove the collision object from the world.
  // ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  // std::vector<std::string> object_ids;
  // object_ids.push_back(collision_object.id);
  // planning_scene_interface.removeCollisionObjects(object_ids);

  // // Show text in RViz of status
  // visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  // visual_tools.trigger();

  // /* Wait for MoveGroup to recieve and process the attached collision object message */
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

  // // END_TUTORIAL

  ros::waitForShutdown();
  return 0;
}