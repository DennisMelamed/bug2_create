/*
 * Title: Contact Sensor Based Algorithm for iRobot Create Navigation Around Obstacles to Goal States
 * Author: Dennis Melamed (University of Minnesota)
 * 
 * Based off of the pathfinding algorithm know as "Bug 2." 
 * 
 * Overall Process:
 * -Calculate shortest distance to target, record the angle from the target to a zero angle position (this describes the line to the target, aka the "m-line")
 * -Drive forward until the target is reached or a collision is detected
 * -If a collision is detected, follow the edge of the object collided against clockwise until the m-line is regained
 * -Continue driving towards target following the m-line, repeat object circumnavigation as required until target reached.
 * 
 * Overall Process:
 * -An angle to the goal position is found and rotated to
 * -The robot moves forward until a collision is detected or the goal position is reached
 * -if a collision is detected:
 * 		-an angle respresenting the line from the current position to the goal position is stored
 * 		-the robot drives back a short distance, 
 * 		-rotates 90 degrees counter-clockwise,
 * 		-drives forward a robot-length,
 * 		-rotates 90 degrees clockwise,
 * 		-drives forward checking for a collision
 * 		-if a collision is detected:
 * 			- the above process repeats itself until a collision is not detected
 * 		-if there is no collision after a certain distance:
 * 			- this indicates the robot has gone off an edge	
 * 			
 *
 */

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


//holds the current position of the robot, its current orientation
double current_pos_x;
double current_pos_y;
double current_theta;
//bug2 algorithm needs to store a line from the origin to the goal position, so the angle of the line the robot is currently on is recorded here
double current_pos_theta;

//holds goal states for various parts of the algorithm

//overall position goal
double goal_x = 1;
double goal_y = 4;
//used when the robot is spinning in place to describe the desired end orientation
double goal_theta;

//how fast the robot moves
double rot_vel=.2;
double drive_speed = .1;

//allowable errors to account for the robot coasting a bit after being sent one stop command
double angle_error = .05;
double angle_pos_error = .009;
double distance_error = .1;

//used during rotations and various translations to hold the beginning state
double start_x;
double start_y;
double start_theta;
//holds how far the robot has moved during rotations and translations
double dist_traveled_x = 0;
double dist_traveled_y = 0;
double dist_traveled = 0;

//holds where the robot began to test around an obstacle so it doesn't immediately think its found the m-line again
double started_test_x;
double started_test_y;

double robot_radius = 0.16495;

//distances used during testing
double test_distance = robot_radius*2;
double backup_distance = .15;

//temporarily holds rotational speed and forward speed during some operations
double rot;
double forward;

//holds the calculated upper and lower error bounds for a specific motion
double uv;
double lv;

//the number of steps in the testing routine, used to loop back through them after finishing one test cycle
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

void setGoalThetaFromCurrent()
{
	goal_theta = atan2((goal_y-current_pos_y),(goal_x-current_pos_x));  
	goal_theta = angles::normalize_angle_positive(goal_theta);
}

void not_testing()
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
	else if (!(current_pos_x <= goal_x+distance_error && current_pos_x >= goal_x-distance_error) || 
			!(current_pos_y <= goal_y+distance_error && current_pos_y >= goal_y-distance_error))
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
	else if((current_pos_x <= goal_x+distance_error && current_pos_x >= goal_x-distance_error) || (current_pos_y <= goal_y+distance_error && current_pos_y >= goal_y-distance_error))
	{
		setGoalThetaFromCurrent();
	}
	if(bump_sensor)
	{
		testing = true;
		msg.linear.x = 0;
		started_test_x = current_pos_x;
		started_test_y = current_pos_y;
	}
}

void testing_step0()
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


	if((backup_distance-distance_error < dist_traveled && dist_traveled < backup_distance+distance_error) || bump_sensor)
	{
		count++;					
		initialized = false;
		msg.linear.x = 0;
	}
}

void testing_step1()
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

void testing_step2()
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

void testing_step3()
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

void testing_step4()
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

		msg.linear.x = 0;
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

void testing_step5()
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

void tester(int step_number)
{
	if(step_number == 0)
	{
		testing_step0();

	}

	else if(step_number ==1)
	{
		testing_step1();			
	}

	else if(step_number == 2)
	{
		testing_step2();
	}

	else if(step_number ==3)
	{
		testing_step3();		
	}

	else if(step_number == 4)
	{
		testing_step4();		
	}

	if(step_number == 5)
	{
		testing_step5();
	}

	uv = greatest(angles::normalize_angle_positive(goal_theta+angle_pos_error), angles::normalize_angle_positive(goal_theta-angle_pos_error));
	lv = least(angles::normalize_angle_positive(goal_theta+angle_pos_error), angles::normalize_angle_positive(goal_theta-angle_pos_error));

	current_pos_theta = atan2(current_pos_y, current_pos_x);
	current_pos_theta = angles::normalize_angle_positive(current_pos_theta);

	if (moved && goal_x>current_pos_x && current_pos_x> 0 && goal_y>current_pos_y && current_pos_y>0 &&
			((uv < current_pos_theta && current_pos_theta <= uv + 2*angle_pos_error) || (lv > current_pos_theta && current_pos_theta > (lv - 2*angle_pos_error))))
	{
		testing = false;
		moved = false;
		setGoalThetaFromCurrent();

	}
	ROS_INFO("goal_pos_theta: %f, current_pos_theta: %f, uv: %f, lv %f, current truth: %d", goal_theta, 
			current_pos_theta, uv, lv ,((uv < current_pos_theta && current_pos_theta <= uv + 2*angle_pos_error) || 
					(lv > current_pos_theta && current_pos_theta > (lv - 2*angle_pos_error))));


	ROS_INFO("The state of RESET is: %d", reset);
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
	setGoalThetaFromCurrent();	  
	//    goal_theta = atan2((goal_y-current_pos_y),(goal_x-current_pos_x));  
	//     goal_theta = angles::normalize_angle_positive(goal_theta);

	while(ros::ok())
	{
		if(bump_sensor)
		{
			msg.linear.x = 0;
		}

		if(!testing)
		{
			not_testing();
		}

		if(testing)
		{
			tester(count%steps);			
		}

		if(bump_sensor)
		{
			msg.linear.x = 0;
		}		

		//Publish the message
		pub.publish(msg);

		//Delays untill it is time to send another message
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

