#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  ros::Rate rate(10.0);
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "camera";

  tf2::Quaternion q_orig(-0.0146, 0.99820, 0.04018, -0.04189), q_new;
  q_new = q_orig.inverse();
  
  while (node.ok())
  {  
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.transform.translation.x = 0.326;
    transformStamped.transform.translation.y = 0.265;
    transformStamped.transform.translation.z = 1.915;
    transformStamped.transform.rotation.x = q_new.x();
    transformStamped.transform.rotation.y = q_new.y();
    transformStamped.transform.rotation.z = q_new.z();
    transformStamped.transform.rotation.w = q_new.w();

    br.sendTransform(transformStamped);

    rate.sleep();
  }
  return 0;
}