

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

double current_pos_x;
double current_pos_y;
double current_theta;
double goal_x = 3;
double goal_y = 3;
double goal_theta;
double rot_vel=.5;
double drive_speed = .5;
double angle_error = .09;
double distance_error = .09;

double start_x;
double start_y;
double start_theta;

double test_distance = .1;

double dist_traveled_x = 0;
double dist_traveled_y =0;
double dist_traveled = 0;

int count = 0;

bool initialized = false;
bool bump_sensor = false;
bool testing = false;

geometry_msgs::Twist msg;


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
   //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
//   ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   //ROS_INFO("current_theta: %f", current_theta);
   //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}

void bumpCallback(const gazebo_msgs::ContactsState& msg)
{

	if(!msg.states.empty())
	{
		
		bump_sensor = true;
//		ROS_INFO("bump detected");
	}
	else
	{
		bump_sensor = false;
	//	ROS_INFO("no contact");
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
     
	//stores the m-line for bug 2 algorithm
     double slope = ((goal_y-current_pos_y)/(goal_x-current_pos_y));
     goal_theta = atan2((goal_x-current_pos_x),(goal_y-current_pos_y));  
     goal_theta = angles::normalize_angle_positive(goal_theta);

       while(ros::ok())
       {
		if(!testing)
		{
			if(!(current_theta <= goal_theta+angle_error && current_theta >= goal_theta-angle_error))	
			{
				msg.linear.x  = 0;
				msg.angular.z = rot_vel;

			//	ROS_INFO("SPINNING: current theta: %f", current_theta);
			//	ROS_INFO("DRIVING: x: %f y: %f", current_pos_x, current_pos_y);
			}
			else if (!(current_pos_x <= goal_x+distance_error && current_pos_x >= goal_x-angle_error) && !(current_pos_y <= goal_y+distance_error && current_pos_y >= goal_y-angle_error))
			{
				msg.linear.x = drive_speed;
				msg.angular.z = 0;
			//	ROS_INFO("SPINNING: current theta: %f", current_theta);
			//	ROS_INFO("DRIVING: x: %f y: %f", current_pos_x, current_pos_y);
			}
			else
			{
				msg.linear.x = 0;
				ROS_INFO("STOPPING");
			}
			if(bump_sensor)
			{
				testing = true;
			}
		}


		if(testing)
		{
			if(count == 0)
			{
				msg.linear.x = -drive_speed;
				if(!initialized)
				{
					double start_x = current_pos_x;
					double start_y = current_pos_y;
					ROS_INFO("x: %f y: %f", start_x, start_y);
					initialized = true;
				}
	
				dist_traveled_x = start_x - current_pos_x;
				dist_traveled_y = start_y - current_pos_y; 	
				ROS_INFO("current_x: %f current_y: %f", current_pos_x, current_pos_y);
				ROS_INFO("traveled_x: %f traveled_y: %f", dist_traveled_x, dist_traveled_y);

				dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    
				ROS_INFO("dist_traveled: %f \n", dist_traveled);

				if(test_distance-distance_error < dist_traveled && dist_traveled <= test_distance+distance_error)
				{
					ROS_INFO("STOPPING");
					count++;					
					initialized = false;
					msg.linear.x = 0;
				}
			}

			if(count ==1)
			{
				msg.linear.z = rot_vel;
				if(!initialized)
				{
					double start_theta = current_theta;
					initialized = true;
				}
				if(start_theta + 1.57 ==current_theta)
				{
					count++;
					initialized = false;
					msg.angular.z = 0;
				}
			}
			
			if(count == 2)
			{
				msg.linear.x = -drive_speed;
				if(!initialized)
				{
					double start_x = current_pos_x;
					double start_y = current_pos_y;
					ROS_INFO("x: %f y: %f", start_x, start_y);
					initialized = true;
				}
	
				dist_traveled_x = start_x - current_pos_x;
				dist_traveled_y = start_y - current_pos_y; 	
				ROS_INFO("current_x: %f current_y: %f", current_pos_x, current_pos_y);
				ROS_INFO("traveled_x: %f traveled_y: %f", dist_traveled_x, dist_traveled_y);

				dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    
				ROS_INFO("dist_traveled: %f \n", dist_traveled);

				if(test_distance-distance_error < dist_traveled && dist_traveled <= test_distance+distance_error)
				{
					ROS_INFO("STOPPING");
					count++;					
					initialized = false;
					msg.linear.x = 0;
				}
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

