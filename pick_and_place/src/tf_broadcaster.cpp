#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped1;
  geometry_msgs::TransformStamped transformStamped2;
  

  ros::Rate rate(10.0);

  transformStamped1.header.frame_id = "camera";
  transformStamped1.child_frame_id = "table";
  
  transformStamped2.header.frame_id = "table";
  transformStamped2.child_frame_id = "base_link";

  
  tf2::Quaternion q(0.999, 0.027, 0.036, -0.012);
  tf2::Quaternion q_rot;
  q_rot.setRPY(0, 0, -3.14/2.0);
  tf2::Quaternion q_new = q_rot*q;
  while (node.ok())
  {  
    transformStamped1.header.stamp = ros::Time::now();
    transformStamped1.transform.translation.x = 0.272;
    transformStamped1.transform.translation.y = 0.177;
    transformStamped1.transform.translation.z = 1.915;
    transformStamped1.transform.rotation.x = q_new.x();
    transformStamped1.transform.rotation.y = q_new.y();
    transformStamped1.transform.rotation.z = q_new.z();
    transformStamped1.transform.rotation.w = q_new.w();

    transformStamped2.header.stamp = ros::Time::now();
    transformStamped2.transform.translation.x = 0.0;
    transformStamped2.transform.translation.y = 0.0;
    transformStamped2.transform.translation.z = 0.025;
    transformStamped2.transform.rotation.x = 0.0;
    transformStamped2.transform.rotation.y = 0.0;
    transformStamped2.transform.rotation.z = 0.0;
    transformStamped2.transform.rotation.w = 1.0;

    br.sendTransform(transformStamped1);
    br.sendTransform(transformStamped2);

    rate.sleep();
  }
  return 0;
}