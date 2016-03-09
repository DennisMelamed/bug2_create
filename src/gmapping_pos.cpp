#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"



int main(int argc, char **argv)
{

  ros::init(argc, argv, "PoseUpdate");


  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>("gmapping_pose", 1000);

  tf::TransformListener listener;
  
  nav_msgs::Odometry pose;

  ros::Rate rate(1.0);
  

  while (n.ok())
  {
    tf::StampedTransform transform;
    try
    {
        //ROS_INFO("Attempting to read pose...");
        listener.lookupTransform("/base_link","/map",ros::Time(0), transform);

        ROS_INFO("Got a transform! x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
        pose.pose.pose.position.x = transform.getOrigin().x();
        pose.pose.pose.position.y = transform.getOrigin().y();
        
        chatter_pub.publish(pose);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Nope! %s", ex.what());
    } 


    rate.sleep();

  }
  ROS_ERROR("I DIED");
  return 0;
}
