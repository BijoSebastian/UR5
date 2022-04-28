// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

void execute_motion(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  // Object to hold manipulator plans 
  moveit::planning_interface::MoveGroupInterface::Plan local_plan;

  // Check result of planning operation
  bool success = false;success = (move_group_interface.plan(local_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Planning status:: %s", success ? "Planning succesfull" : "FAILED");

  if(success)
  {
    // Execute the plan on the robot 
    move_group_interface.move();
    ros::Duration(2.0).sleep();
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_pick_place_tf");
  ros::NodeHandle nh;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //For tf
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP_ARM = "manipulator";

  // The planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

  move_group_interface_arm.setPlanningTime(20.0); //This should result in better optimized plans 
  
  //Wait for setup to complete
  ros::Duration(2.0).sleep();
  
  // Move robot to UP pose, as an intermediate.
  ROS_INFO_NAMED("tutorial", "Move UP");
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("up"));
  execute_motion(move_group_interface_arm);

  for(int i = 0; i<5; i++)
  {
    // Obtain the updated tag pose from tf
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("table", "tag_6",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // Get the current robot pose to copy wrist_3_link's orientation 
    ROS_INFO_NAMED("tutorial", "Place the TCP above a desired point");
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("wrist_3_link");

    // Compute the target pose: same wrist 3 orientation as before 
    // but position will be based on the april tag position 
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = transformStamped.transform.translation.x;
    target_pose1.position.y = transformStamped.transform.translation.y - 0.25;
    target_pose1.position.z = transformStamped.transform.translation.z - 0.05;
    
    // Move robot to target pose
    std::cout<<"Desired pose :: x: "<<target_pose1.position.x<<"y: "<<target_pose1.position.y<<"z: "<<target_pose1.position.z<<"\n";
    move_group_interface_arm.setPoseTarget(target_pose1);
    execute_motion(move_group_interface_arm);
    
    // Move robot to UP pose again, as an intermediate pose
    // this gives you time to move the april tag to a new position
    ROS_INFO_NAMED("tutorial", "Move UP");
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("up"));
    execute_motion(move_group_interface_arm);
  }

  ros::shutdown();
  return 0;
}