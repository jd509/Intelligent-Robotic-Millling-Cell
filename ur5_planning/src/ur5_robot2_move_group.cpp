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
    // std::vector<double> target_joint_angles = {0.8157, -1.7009, -1.9787, -2.5341, 1.6315, -0.1041};
    // move_to_configuration(target_joint_angles);

    //ROS publishers and subscribers
    receive_data_from_coord_sub = node_handle_rob2.subscribe("/command_rob_2", 1000, &Move_Group_Robot_2::perform_actions,this);
    send_update_pub = node_handle_rob2.advertise<std_msgs::String>("/rob2_to_coord", 1000);

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
    std::vector<double> target_joint_angles = {0.8157, -1.7009, -1.9787, -2.5341, 1.6315, -0.1041};
    move_to_configuration(target_joint_angles);
    send_update("robot_2_intialization_complete");
  }
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

void Move_Group_Robot_2::move_to_configuration(std::vector<double>& joint_angles)
{
  ur5_robot2_group_ptr->setStartStateToCurrentState();

  ur5_robot2_group_ptr->setJointValueTarget(joint_angles);

  bool success = (ur5_robot2_group_ptr->plan(ur5_robot2_goal_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ur5_robot2_group_ptr->move();
  std::cout<<ur5_robot2_goal_plan.trajectory_.joint_trajectory.points[0]<<std::endl;
  std::cout<<ur5_robot2_goal_plan.trajectory_.joint_trajectory.points.back()<<std::endl;
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