#include "ur5_planning/ur5_robot1_move_group.hpp"


Move_Group_Robot_1::Move_Group_Robot_1()
{
    std::cout<<"Loading Move Group for Ur5 Robot 1 \n";
    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle_rob1};
    
    ur5_robot1_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    joint_model_group = ur5_robot1_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    joint_names = joint_model_group->getVariableNames();
    link_names = joint_model_group->getLinkModelNames();

    // std::cout<<ur5_robot1_group_ptr->getPlanningFrame()<<std::endl;

    for(auto i:joint_names)
    {
      std::cout<<"Joint Name: "<<i<<std::endl;
    }
    // std::vector<double> target_joint_angles = {0.8157, -1.7009, -1.9787, -2.5341, 1.6315, -0.1041};
    // move_to_configuration(target_joint_angles);

    //ROS publishers and subscribers
    receive_data_from_coord_sub = node_handle_rob1.subscribe("/command_rob_1", 1000, &Move_Group_Robot_1::perform_actions,this);
    send_update_pub = node_handle_rob1.advertise<std_msgs::String>("/rob1_to_coord", 1000);
}

void Move_Group_Robot_1::send_update(std::string msg)
{
  update_msg.data = msg;
  send_update_pub.publish(update_msg);
  update_msg.data.clear();
}


void Move_Group_Robot_1::perform_actions(const std_msgs::String& msg)
{
  if(msg.data.compare("start_robot_initialization") == 0)
  {
    std::vector<double> target_joint_angles = {0.8157, -1.7009, -1.9787, -2.5341, 1.6315, -0.1041};
    move_to_configuration(target_joint_angles);
    send_update("robot_1_intialization_complete");
  }
}

void Move_Group_Robot_1::add_robot_table()
{
  //Intializing Collision Object for robot table
  moveit_msgs::CollisionObject robot_table;
  robot_table.header.frame_id = ur5_robot1_group_ptr->getPlanningFrame();
  robot_table.id = "robot_table_1";

  //Creating a mesh from stl of table
  std::string mesh_file_for_table = "package://process_visualizer/resources/table.stl";
  shapes::Mesh *table_mesh = shapes::createMeshFromResource(mesh_file_for_table);

  //Defining shape message
  shape_msgs::Mesh table_mesh_;
  shapes::ShapeMsg table_mesh_msg;
  shapes::constructMsgFromShape(table_mesh, table_mesh_msg);
  table_mesh_ = boost::get<shape_msgs::Mesh>(table_mesh_msg);

  //Position of table
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.orientation.x = 0.0;
  table_pose.orientation.y = 0.0;
  table_pose.orientation.z = 0.0;

  table_pose.position.x = 0.87;
  table_pose.position.y = 0.57;
  table_pose.position.z = 0.005;

  //Adding mesh to collision object
  robot_table.meshes.push_back(table_mesh_);
  robot_table.mesh_poses.push_back(table_pose);
  robot_table.operation = robot_table.ADD;

  collision_objects.push_back(robot_table);

  planning_scene_interface.addCollisionObjects(collision_objects);
}

void Move_Group_Robot_1::move_to_configuration(std::vector<double>& joint_angles)
{
  // moveit::core::RobotStatePtr current_state = ur5_robot1_group_ptr->getCurrentState();
  
  // moveit::planning_interface::MoveGroupInterface::Plan abb_rob1_plan;
  // std::vector<double> target_joint_angles = {0.8157, -1.7009, -1.9787, -2.5341, 1.6315, -0.1041};
  // std::vector<double> target_joint_angles_2 = {0.7500, -1.7009, -1.9787, -2.5341, 1.6315, -0.0041};

  ur5_robot1_group_ptr->setStartStateToCurrentState();

  ur5_robot1_group_ptr->setJointValueTarget(joint_angles);

  bool success = (ur5_robot1_group_ptr->plan(ur5_robot1_goal_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ur5_robot1_group_ptr->move();
  std::cout<<ur5_robot1_goal_plan.trajectory_.joint_trajectory.points[0]<<std::endl;
  std::cout<<ur5_robot1_goal_plan.trajectory_.joint_trajectory.points.back()<<std::endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_robot1_planning");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  Move_Group_Robot_1 rob1_obj;
        
  rob1_obj.add_robot_table();

  std::cout<<"Robot Table added as a Collision Object"<<std::endl;

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