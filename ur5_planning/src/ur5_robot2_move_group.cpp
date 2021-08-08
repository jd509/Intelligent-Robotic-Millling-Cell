#include "ur5_planning/ur5_robot2_move_group.hpp"


Move_Group_Robot_2::Move_Group_Robot_2()
{
    std::cout<<"Loading Move Group for Ur5 Robot 2 \n";
    moveit::planning_interface::MoveGroupInterface::Options Opt{PLANNING_GROUP, ROBOT_DESCRIPTION, node_handle_rob2};
    
    ur5_robot2_group_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(Opt);
    joint_model_group = ur5_robot2_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    joint_names = joint_model_group->getVariableNames();
    link_names = joint_model_group->getLinkModelNames();

    for(auto i:joint_names)
    {
      std::cout<<"Joint Name: "<<i<<std::endl;
    }
    ur5_robot2_group_ptr->setGoalTolerance(0.01);
    ur5_robot2_group_ptr->allowReplanning(true);
    //ROS publishers and subscribers
    receive_data_from_coord_sub = node_handle_rob2.subscribe("/command_rob_2", 1000, &Move_Group_Robot_2::perform_actions,this);
    send_update_pub = node_handle_rob2.advertise<std_msgs::String>("/rob2_to_coord", 1000);
    planning_scene_diff_publisher = node_handle_rob2.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

}

void Move_Group_Robot_2::send_update(std::string msg)
{
  update_msg.data = msg;
  send_update_pub.publish(update_msg);
  update_msg.data.clear();
}


void Move_Group_Robot_2::perform_actions(const std_msgs::String& msg)
{
  if(msg.data.compare("start_robot_initialization") == 0)
  {
    std::vector<double> target_joint_angles = {1.51844, -1.53589, 1.51844, -1.58825, -1.58825, 0.0001};
    move_to_configuration(target_joint_angles);
    send_update("robot_2_intialization_complete");
    //place_fr_milling();
  }
  if(msg.data.compare("start_pickup_rob2")==0)
  {
    pick();
  }
  if(msg.data.compare("Crazing")==0)
  {
    place_bin1();
  }
  if(msg.data.compare("Inclusion")==0)
  {
    place_bin2();
  }
  if(msg.data.compare("No_Defect")==0)
  {
    place_fr_milling();
  }
}

std::string Move_Group_Robot_2::intToString (int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
  }


void Move_Group_Robot_2::add_robot_table()
{
  //Intializing Collision Object for robot table
  moveit_msgs::CollisionObject robot_table;
  robot_table.header.frame_id = ur5_robot2_group_ptr->getPlanningFrame();
  robot_table.id = "robot_table_2";

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
  table_pose.position.y = -0.83;
  table_pose.position.z = 0.005;

  //Adding mesh to collision object
  robot_table.meshes.push_back(table_mesh_);
  robot_table.mesh_poses.push_back(table_pose);
  robot_table.operation = robot_table.ADD;

  collision_objects.push_back(robot_table);

  planning_scene_interface.addCollisionObjects(collision_objects);
}

void Move_Group_Robot_2::pick()
{
  //Pickup Location
  double x = 1.1, y = -0.08, z = 0.525;

  //Setting Pre Grasp Pose
  geometry_msgs::Pose target_pose; 
  target_pose = ur5_robot2_group_ptr->getCurrentPose().pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z + 0.1;
  std::cout<<"Target Pose: "<<target_pose;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  //grapsing position
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose);

  target_pose.position.z = z;
  std::cout<<"Target Pose: "<<target_pose;
  waypoints.push_back(target_pose);

  ur5_robot2_group_ptr->setMaxVelocityScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  
  target_pose.position.z = z+0.1;
  waypoints.push_back(target_pose);
  
  double fraction = ur5_robot2_group_ptr->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);




  ur5_robot2_group_ptr->execute(trajectory);
  // ros::Duration(1.0).sleep();
  ur5_robot2_group_ptr->setStartStateToCurrentState();


  //Attaching object to robot
  send_update("attached_rob2");

}

void Move_Group_Robot_2::place_bin1()
{
  //pre place location
  geometry_msgs::Pose pre_place_pose;
  geometry_msgs::Pose target_pose; 
  target_pose = ur5_robot2_group_ptr->getCurrentPose().pose;
  target_pose.position.x = bin_1_pos[bin1_wp][0];
  target_pose.position.y = bin_1_pos[bin1_wp][1];
  target_pose.position.z = bin_1_pos[bin1_wp][2] + 0.2;
  std::cout<<"Target Pose: "<<target_pose;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  //place position
  target_pose.position.z = bin_1_pos[bin1_wp][2] +0.01;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  send_update("workpiece_placed");
//  Adding stoppage time after placing
  ros::Duration(3.0).sleep(); 


  //pre-retract pos
  target_pose.position.z = bin_1_pos[bin1_wp][2] +0.2;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  //Retract
  std::vector<double> target_joint_angles = {1.51844, -1.53589, 1.51844, -1.58825, -1.58825, 0.0001};
  move_to_configuration(target_joint_angles);
  ur5_robot2_group_ptr->setStartStateToCurrentState();
  bin1_wp ++;
}

void Move_Group_Robot_2::place_bin2()
{
  //pre place location
  geometry_msgs::Pose pre_place_pose; 
  geometry_msgs::Pose target_pose;
  target_pose = ur5_robot2_group_ptr->getCurrentPose().pose;
  target_pose.position.x = bin_2_pos[bin2_wp][0];
  target_pose.position.y = bin_2_pos[bin2_wp][1];
  target_pose.position.z = bin_2_pos[bin2_wp][2] + 0.2;
  std::cout<<"Target Pose: "<<target_pose;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  //place position
  target_pose.position.z = bin_2_pos[bin2_wp][2] +0.01;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  send_update("workpiece_placed");
//  Adding stoppage time after placing
  ros::Duration(4.0).sleep(); 


  //pre-retract pos
  target_pose.position.z = bin_2_pos[bin2_wp][2] +0.2;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  //Retract
  std::vector<double> target_joint_angles = {1.51844, -1.53589, 1.51844, -1.58825, -1.58825, 0.0001};
  move_to_configuration(target_joint_angles);
  ur5_robot2_group_ptr->setStartStateToCurrentState();
  bin2_wp ++;
}

void Move_Group_Robot_2::place_fr_milling()
{
  double x = 0.3, y = -0.7, z=0.6; //location for placing on milling table
  //pre place location
  std::vector<double> target_joint_angles1 = {2.93215, -1.53589, 1.51844, -1.58825, -1.58825, 0.0001};
  move_to_configuration(target_joint_angles1);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  //pre place location
  geometry_msgs::Pose target_pose;
  target_pose = ur5_robot2_group_ptr->getCurrentPose().pose;
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z + 0.2;
  std::cout<<"Target Pose: "<<target_pose;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  //place position
  target_pose.position.z = z + 0.05;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  send_update("workpiece_placed");
//  Adding stoppage time after placing
  ros::Duration(4.0).sleep(); 


  //pre-retract pos
  target_pose.position.z = z + 0.2;
  move_to_pose(target_pose);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  //Retract
  std::vector<double> target_joint_angles = {1.51844, -1.53589, 1.51844, -1.58825, -1.58825, 0.0001};
  move_to_configuration(target_joint_angles);
  ur5_robot2_group_ptr->setStartStateToCurrentState();

}

void Move_Group_Robot_2::move_to_configuration(std::vector<double>& joint_angles)
{
  moveit_msgs::PlanningScene planning_scene;
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  ur5_robot2_group_ptr->setJointValueTarget(joint_angles);

  bool success = (ur5_robot2_group_ptr->plan(ur5_robot2_goal_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ur5_robot2_group_ptr->move();

  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  std::cout<<ur5_robot2_goal_plan.trajectory_.joint_trajectory.points[0]<<std::endl;
  std::cout<<ur5_robot2_goal_plan.trajectory_.joint_trajectory.points.back()<<std::endl;
}

void Move_Group_Robot_2::move_to_pose(geometry_msgs::Pose target_p)
{
  moveit_msgs::PlanningScene planning_scene;
  ur5_robot2_group_ptr->setStartStateToCurrentState();
  geometry_msgs::Pose start_pose;
  start_pose = ur5_robot2_group_ptr->getCurrentPose().pose;
  std::cout<<"Start Pose: "<<start_pose;

  ur5_robot2_group_ptr->setPoseTarget(target_p);
  std::cout<<"Target Pose: "<<target_p;

  bool success = (ur5_robot2_group_ptr->plan(ur5_robot2_goal_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ur5_robot2_group_ptr->move();
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_robot2_planning");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  Move_Group_Robot_2 rob2_obj;
        
  rob2_obj.add_robot_table();


   ros::waitForShutdown();
  return 0;
}