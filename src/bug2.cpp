

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
double current_pos_theta;

double goal_x = 1;
double goal_y = 4;
double goal_theta;
double rot_vel=.2;
double drive_speed = .1;

double angle_error = .15;
double angle_pos_error = .009;
double distance_error = .1;
double distance_error_small = .00001;
double slope_error = .01;

double start_x;
double start_y;
double start_theta;

double started_test_x;
double started_test_y;

double robot_radius = 0.16495;

double test_distance = robot_radius*2;
double backup_distance = .05;

double dist_traveled_x = 0;
double dist_traveled_y = 0;
double dist_traveled = 0;

double rot;
double forward;

double uv;
double lv;


int count = 0;
int steps = 6;

bool initialized = false;
bool bump_sensor = false;
bool testing = false;
bool reset = false;
bool moved = false;

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

double greatest(double a, double b)
{
	if(a>b)
	{
		return a;
	}
	if(b<a)
	{
		return b;
	}
	return 0;
}

double least(double a, double b)
{
	if(a<b)
	{
		return a;
	}
	if(b<a)
	{
		return b;
	}
	return 0;
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
     double slope = ((goal_y-current_pos_y)/(goal_x-current_pos_x));
     goal_theta = atan2((goal_y-current_pos_y),(goal_x-current_pos_x));  
     goal_theta = angles::normalize_angle_positive(goal_theta);

       while(ros::ok())
       {
		if(!testing)
		{
			if(!(current_theta <= goal_theta+angle_error && current_theta >= goal_theta-angle_error))	
			{
				msg.linear.x  = 0;
		//		if(current_theta < goal_theta)
		//		{
					msg.angular.z = rot_vel;
		//		}
		//		if(current_theta > goal_theta)
		//		{
		//			msg.angular.z = -rot_vel;
		//		}

				ROS_INFO("SPINNING: current theta: %f goal_theta: %f", current_theta, goal_theta);
			//	ROS_INFO("DRIVING: x: %f y: %f", current_pos_x, current_pos_y);
			}
			else if (!(current_pos_x <= goal_x+distance_error && current_pos_x >= goal_x-distance_error) || !(current_pos_y <= goal_y+distance_error && current_pos_y >= goal_y-distance_error))
			{
				msg.linear.x = drive_speed;
				msg.angular.z = 0;
				ROS_INFO("SPINNING: current theta: %f", current_theta);
				ROS_INFO("DRIVING: x: %f y: %f, goal_x: %f, goal_y: %f", current_pos_x, current_pos_y, goal_x, goal_y);
			}
			else if((current_pos_x <= goal_x+distance_error && current_pos_x >= goal_x-distance_error) && (current_pos_y <= goal_y+distance_error && current_pos_y >= goal_y-distance_error))
			{
				msg.linear.x = 0;
//				ROS_INFO("STOPPING");
			}
			if(bump_sensor)
			{
				testing = true;
				msg.linear.x = 0;
				started_test_x = current_pos_x;
				started_test_y = current_pos_y;
			}
		}


		if(testing)
		{
			if(count%steps == 0)
			{
				ROS_INFO("STEP 0");
				msg.linear.x = -drive_speed;
				if(!initialized)
				{
					start_x = current_pos_x;
					start_y = current_pos_y;
//					ROS_INFO("x: %f y: %f", start_x, start_y);
					initialized = true;
				}
	
				dist_traveled_x = start_x - current_pos_x;
				dist_traveled_y = start_y - current_pos_y; 	
//				ROS_INFO("current_x: %f current_y: %f", current_pos_x, current_pos_y);
//				ROS_INFO("traveled_x: %f traveled_y: %f", dist_traveled_x, dist_traveled_y);

				dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    
//				ROS_INFO("dist_traveled: %f \n", dist_traveled);

				
				if(backup_distance-distance_error < dist_traveled && dist_traveled < backup_distance+distance_error)
				{
					count++;					
					initialized = false;
					msg.linear.x = 0;
				}
			}

			else if(count%steps ==1)
			{
				ROS_INFO("Step 1");
				msg.angular.z = rot_vel;
				if(!initialized)
				{
					start_theta = current_theta;
					initialized = true;
//					ROS_INFO("start_theta: %f", start_theta);
				}
				
				uv = greatest(angles::normalize_angle_positive(start_theta+1.57+angle_error), angles::normalize_angle_positive(start_theta+1.57-angle_error));
				lv = least(angles::normalize_angle_positive(start_theta+1.57+angle_error), angles::normalize_angle_positive(start_theta+1.57-angle_error));
				
				ROS_INFO("current_theta: %f, start_theta: %f, boundaries: %f , %f", current_theta, start_theta, lv, uv);
				if((uv < current_theta && current_theta <= uv + 2*angle_error) || (lv > current_theta && current_theta > (lv - 2*angle_error)))
				{
					count++;
					initialized = false;
					msg.angular.z = 0;
					//ROS_INFO("STOPPING STEP 1");
				}
			}
			
			else if(count%steps == 2)
			{
				ROS_INFO("STEP 2");
				msg.linear.x = drive_speed;
				if(!initialized)
				{
					start_x = current_pos_x;
					start_y = current_pos_y;
//					ROS_INFO("x: %f y: %f", start_x, start_y);
					initialized = true;
				}
	
				dist_traveled_x = start_x - current_pos_x;
				dist_traveled_y = start_y - current_pos_y; 	
//				ROS_INFO("current_x: %f current_y: %f", current_pos_x, current_pos_y);
//				ROS_INFO("traveled_x: %f traveled_y: %f", dist_traveled_x, dist_traveled_y);

				dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    
//				ROS_INFO("dist_traveled: %f \n", dist_traveled);

				if(bump_sensor)
				{
					count = 0;
					msg.linear.x = 0;
					initialized = false;
				}
				
				if(test_distance-distance_error < dist_traveled && dist_traveled <= test_distance+distance_error)
				{
					moved = true;
					count++;					
					initialized = false;
					msg.linear.x = 0;
				}
			}
			else if(count%steps ==3)
			{
				ROS_INFO("Step 3");
				msg.angular.z = -rot_vel;
				if(!initialized)
				{
					start_theta = current_theta;
					initialized = true;
				}
				uv = greatest(angles::normalize_angle_positive(start_theta-1.57+angle_error), angles::normalize_angle_positive(start_theta-1.57-angle_error));
				lv = least(angles::normalize_angle_positive(start_theta-1.57+angle_error), angles::normalize_angle_positive(start_theta-1.57-angle_error));
				
				ROS_INFO("current_theta: %f, start_theta: %f, boundaries: %f , %f", current_theta, start_theta, lv, uv);
				if((uv < current_theta && current_theta <= uv + 2*angle_error) || (lv > current_theta && current_theta > (lv - 2*angle_error)))
				{
					count++;
					initialized = false;
					msg.angular.z = 0;
//					ROS_INFO("we've hit this spin back step (3)");
				}
			}
			else if(count%steps == 4)
			{
				ROS_INFO("step 4...");	
				if(!initialized)
				{
					start_x = current_pos_x;
					start_y = current_pos_y;
//					ROS_INFO("x: %f y: %f", start_x, start_y);
					initialized = true;
					forward = drive_speed;
					rot = 0;
						
					start_theta = current_theta;
				}
					
				msg.linear.x = forward;
				msg.angular.z = rot;
				dist_traveled_x = start_x - current_pos_x;
				dist_traveled_y = start_y - current_pos_y; 	
				//ROS_INFO("current_x: %f current_y: %f", current_pos_x, current_pos_y);
				//ROS_INFO("traveled_x: %f traveled_y: %f", dist_traveled_x, dist_traveled_y);

				dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    
				//ROS_INFO("dist_traveled: %f \n", dist_traveled);
				
				if(bump_sensor && !reset)
				{
						count=0;
						initialized = false;
						reset = false;
						msg.angular.z = 0;
				}
				
				
				if(backup_distance-distance_error + robot_radius < dist_traveled && dist_traveled <= backup_distance+distance_error + robot_radius)
				{		
					
					if(!bump_sensor && !reset)
					{
						reset = true;
						initialized = false;
					}
					if (reset)
					{
						forward = 0;
						rot = -rot_vel;
					}
				}
				uv = greatest(angles::normalize_angle_positive(start_theta-1.57+angle_error), angles::normalize_angle_positive(start_theta-1.57-angle_error));
				lv = least(angles::normalize_angle_positive(start_theta-1.57+angle_error), angles::normalize_angle_positive(start_theta-1.57-angle_error));
				
				ROS_INFO("current_theta: %f, start_theta: %f, boundaries: %f , %f", current_theta, start_theta, lv, uv);
				if((uv < current_theta && current_theta <= uv + 2*angle_error) || (lv > current_theta && current_theta > (lv - 2*angle_error)))
				{
					count++;
					initialized = false;
					reset = false;
					msg.angular.z = 0;
				}	
			}
			
			if(count%steps == 5)
			{
				ROS_INFO("STEP 5");
				msg.linear.x = drive_speed;
				if(!initialized)
				{
					start_x = current_pos_x;
					start_y = current_pos_y;
//					ROS_INFO("x: %f y: %f", start_x, start_y);
					initialized = true;
				}
	
				dist_traveled_x = start_x - current_pos_x;
				dist_traveled_y = start_y - current_pos_y; 	
//				ROS_INFO("current_x: %f current_y: %f", current_pos_x, current_pos_y);
//				ROS_INFO("traveled_x: %f traveled_y: %f", dist_traveled_x, dist_traveled_y);

				dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    
//				ROS_INFO("dist_traveled: %f \n", dist_traveled);

				if(bump_sensor)
				{
					count = 0;
					msg.linear.x = 0;
					initialized = false;
				}
				
				if(test_distance-distance_error < dist_traveled && dist_traveled <= test_distance+distance_error)
				{
					
					count++;					
					initialized = false;
					msg.linear.x = 0;
				}
			}
			
			uv = greatest(angles::normalize_angle_positive(goal_theta+angle_pos_error), angles::normalize_angle_positive(goal_theta-angle_pos_error));
			lv = least(angles::normalize_angle_positive(goal_theta+angle_pos_error), angles::normalize_angle_positive(goal_theta-angle_pos_error));
			
			current_pos_theta = atan2(current_pos_y, current_pos_x);
			current_pos_theta = angles::normalize_angle_positive(current_pos_theta);
			
			if (moved && goal_x>current_pos_x && current_pos_x> 0 && goal_y>current_pos_y && current_pos_y>0 && ((uv < current_pos_theta && current_pos_theta <= uv + 2*angle_pos_error) || (lv > current_pos_theta && current_pos_theta > (lv - 2*angle_pos_error))))
			{
				testing = false;
				moved = false;
				
			}
			ROS_INFO("goal_pos_theta: %f, current_pos_theta: %f, uv: %f, lv %f, current truth: %d", goal_theta, current_pos_theta, uv, lv ,((uv < current_pos_theta && current_pos_theta <= uv + 2*angle_pos_error) || (lv > current_pos_theta && current_pos_theta > (lv - 2*angle_pos_error))));
			
			
			ROS_INFO("The state of RESET is: %d", reset);
						
		}
		

           //Publish the message
           pub.publish(msg);

          //Delays untill it is time to send another message
          rate.sleep();
	  ros::spinOnce();
       }
return 0;
}

