

#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h>
#include "nav_msgs/Odometry.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h> 
#include <gazebo_msgs/ContactsState.h>
#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

float x_direction=2;
double current_pos_x;
double current_pos_y;
double current_theta;
double goal_x = 3;
double goal_y = 3;
double goal_theta;
double wheel_radius = .5380/2;
double rot_vel=.2;
double drive_speed = .5;
double x_current_to_goal;
double y_current_to_goal;
double current_to_goal;
double angle_error = .05;

bool bump_sensor;
bool initialized = false;
bool testing = false;

geometry_msgs::Twist msg;

void turn(double goal_theta, geometry_msgs::Twist msg);

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

   current_pos_x = msg->pose.pose.position.x;
   current_pos_y = msg->pose.pose.position.y;
   

   tf::Quaternion quat;
   tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
   double roll;
   double pitch;
   tf::Matrix3x3(quat).getRPY(roll, pitch, current_theta);
   current_theta = angles::normalize_angle_positive(current_theta);
 
  //ROS_INFO("Seq: [%d]", msg->header.seq);
   ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
//   ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   ROS_INFO("current_theta: %f", current_theta);
   //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

void bumpCallback(const gazebo_msgs::ContactsState& msg)
{

	if(!msg.states.empty())
	{
		
		bump_sensor = true;
		ROS_INFO("bump detected");
	}
	else
	{
		bump_sensor = false;
		ROS_INFO("no contact");
	}
}


int main(int argc, char **argv)
{
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "create");
     ros::NodeHandle nh;

     //Ceates the publisher, and tells it to publish
     //to the /cmd_vel topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
     ros::Subscriber odom =nh.subscribe("odom", 100, odomCallback);
     ros::Subscriber bump =nh.subscribe("base_bumper", 100, bumpCallback);
     ros::Rate rate(10);
   

       while(ros::ok())
       {

	if(!testing && !bump_sensor)
	{
		if(!initialized)
		{
			double goal_theta = atan2(y_goal,x_goal);
			goal_theta =angles::normalize_angle_positive(goal_theta); 
			double distance = wheel_radius * goal_theta;

			double rot_time  = distance/rot_vel;
			ROS_INFO("goal theta: %f", goal_theta);

			current_to_goal = goal_theta - current_theta;
			ROS_INFO("current_to_goal: %f",current_to_goal);
	

			if (angle_error <= current_to_goal || current_to_goal <= -angle_error)
			{
				ROS_INFO("SPINNING");
				msg.angular.z=rot_vel;
			}
			else
			{
				ROS_INFO("I SHOULD BE STOPPING");
				msg.angular.z = 0;
				intialized = true;
			}
		}
		else if ((current_pos_x ! = goal_x && current_pos_y != goal_y)
		{
			msg.angular.z = 0;
			msg.linear.x = drive_speed;
		} 
		else
		{
			msg.linear.x = 0;
	
		}
	}	
	 //Declares the message to be sent
           
           //Publish the message
           pub.publish(msg);

          //Delays untill it is time to send another message
          rate.sleep();
	  ros::spinOnce();
       }
return 0;
}

