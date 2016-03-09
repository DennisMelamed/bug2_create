#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	ROS_INFO("%f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y);

	
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "PoseUpdate");


  ros::NodeHandle n;


  

  ros::Subscriber amcl =n.subscribe("amcl_pose", 100, amclCallback);
  
  nav_msgs::Odometry pose;

  ros::Rate rate(1.0);
  

  while (n.ok())
  {
   
   rate.sleep();

  }
  ROS_ERROR("I DIED");
  return 0;
}
