// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void execute_motion(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  // Object to hold manipulator plans 
  moveit::planning_interface::MoveGroupInterface::Plan local_plan;

  // Check result of planning operation
  bool success = false;success = (move_group_interface.plan(local_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Planning status:: %s", success ? "Planning succesfull" : "FAILED");

  if(success)
  {
    move_group_interface.move();
    ros::Duration(2.0).sleep();
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_pick_place");
  ros::NodeHandle nh;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP_ARM = "manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  // The planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);
  
  // Object to add the collision object to the Planning Scene Monitor
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Collision object
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();

  collision_object.id = "table";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2.0;
  primitive.dimensions[1] = 2.0;
  primitive.dimensions[2] = 0.05;
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.0;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.applyCollisionObjects(collision_objects);
  ros::Duration(2.0).sleep();

  // 1. Move to UP position
  ROS_INFO_NAMED("tutorial", "Move UP");
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("up"));
  execute_motion(move_group_interface_arm);
  
  
  // 2. Place the TCP (Tool Center Point, the tip of the robot) above a desired point
  ROS_INFO_NAMED("tutorial", "Place the TCP above a desired point");
  geometry_msgs::PoseStamped current_pose;
  current_pose = move_group_interface_arm.getCurrentPose("wrist_3_link");
  
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = current_pose.pose.orientation;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.5;
  target_pose1.position.z = 0.5;

  move_group_interface_arm.setPoseTarget(target_pose1);
  execute_motion(move_group_interface_arm);

  // 3. Open gripper
  /*
  ROS_INFO_NAMED("tutorial", "Open gripper");
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open_grip"));
  execute_motion(move_group_interface_gripper);
  */

  // 4. Move the TCP close to the object
  ROS_INFO_NAMED("tutorial", "Move TCP close to object");
  target_pose1.position.z = target_pose1.position.z - 0.2;
  move_group_interface_arm.setPoseTarget(target_pose1);
  execute_motion(move_group_interface_arm);
  
  // 5. Close the  gripper
  /*
  ROS_INFO_NAMED("tutorial", "Close gripper");
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("close_grip"));
  execute_motion(move_group_interface_gripper);
  */

  // 6. Move the TCP to above target position
  ROS_INFO_NAMED("tutorial", "Move TCP above target position");
  target_pose1.position.z = target_pose1.position.z + 0.2;
  target_pose1.position.x = target_pose1.position.x - 0.6;
  move_group_interface_arm.setPoseTarget(target_pose1);
  execute_motion(move_group_interface_arm);

  // 7.Open the gripper
  /*
  ROS_INFO_NAMED("tutorial", "Open gripper");
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
  execute_motion(move_group_interface_gripper);
  */

  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  ros::shutdown();
  return 0;
}
