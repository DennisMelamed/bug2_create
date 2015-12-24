/*
 * Title: Contact Sensor Based Implementation of iRobot Create Navigation Around Obstacles to Goal Postions
 * Author: Dennis Melamed (University of Minnesota)
 * 
 * Based off of the pathfinding algorithm know as "Bug 2." 
 * A more complete explanation with visuals (by Howie Choset at Carnegie Mellon University) can be found here: 
 * http://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf
 * 
 * 
 * Overall Process:
 * -Calculate shortest distance to target, record the angle from the target to a zero angle position (this describes the line to the target, aka the "m-line")
 * -Drive along the m-line until the target is reached or a collision is detected
 * -If a collision is detected, follow the edge of the object collided against clockwise until the m-line is regained
 * -Continue driving towards target following the m-line, repeat object circumnavigation as required until target reached.
 * 
 * More Detailed Flow:
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
 * 			- this indicates the robot has gone off an edge of the obstacle
 * 			-The robot drives forward an additional robot-length,
 * 			-rotates 90 degrees clockwise
 * 			-drives forward a robot-lenght
 * 			-begins testing again as though a collision had just been detected
 * 		-Once the robot is at the same angle from the origin as the goal position is, the m-line has been regained
 * 		-The robot rotates to face the goal position and begins driving toward it again
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
double goal_x = -1;
double goal_y = -4;
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
	//xy positions
	current_pos_x = msg->pose.pose.position.x;
	current_pos_y = msg->pose.pose.position.y;

	//orientation
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
	}
	else
	{
		bump_sensor = false;
	}
}

//returns the larger of the two passed doubles
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

//returns the smallest of the two passed doubles
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

//sets the robot's goal theta to point at the goal position
void setGoalThetaFromCurrent()
{
	goal_theta = atan2((goal_y-current_pos_y),(goal_x-current_pos_x));  
	goal_theta = angles::normalize_angle_positive(goal_theta);
}

void drive(bool x) //true for forward, false for backwards
{
	if(x)
	{
		msg.linear.x = drive_speed;
		msg.angular.z = 0;
	}
	else
	{
		msg.linear.x = -drive_speed;
		msg.angular.z = 0;
	}
}

void rotate(bool x) //true for counterclockwise, false for clockwise
{
	if (x)
	{
		msg.linear.x = 0;
		msg.angular.z = rot_vel;
	}
	else
	{
		msg.linear.x = 0;
		msg.angular.z = -rot_vel;
	}
}

void stopDrive()
{
	msg.linear.x = 0;
}

void stopRotate()
{
	msg.angular.z = 0;
}

//actions that occur if the robot is just moving towards the goal and is not testing
void not_testing()
{
	//if the robot isn't pointing the right direction
	if(!(current_theta <= goal_theta+angle_error && current_theta >= goal_theta-angle_error))	
	{
		rotate(true);

		//("SPINNING: current theta: %f goal_theta: %f", current_theta, goal_theta);
	}
	//if we aren't at the goal, move to it
	else if (!(current_pos_x <= goal_x+distance_error && current_pos_x >= goal_x-distance_error) || 
			!(current_pos_y <= goal_y+distance_error && current_pos_y >= goal_y-distance_error))
	{
		drive(true);

		//("DRIVING: x: %f y: %f, goal_x: %f, goal_y: %f", current_pos_x, current_pos_y, goal_x, goal_y);
	}
	
	//if we're at out goal, stop
	else if((current_pos_x <= goal_x+distance_error && current_pos_x >= goal_x-distance_error) && (current_pos_y <= goal_y+distance_error && current_pos_y >= goal_y-distance_error))
	{
		stopDrive();
	}
	
	//if there's been a collision, enter the testing routine
	if(bump_sensor)
	{
		stopDrive();
		started_test_x = current_pos_x;
		started_test_y = current_pos_y;
		testing = true;
	}
}

void testing_step0()
{
	//("STEP 0");
	drive(false);
	
	if(!initialized)
	{
		start_x = current_pos_x;
		start_y = current_pos_y;
		initialized = true;
	}

	dist_traveled_x = start_x - current_pos_x;
	dist_traveled_y = start_y - current_pos_y; 	
	dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    

	if((backup_distance-distance_error < dist_traveled && dist_traveled < backup_distance+distance_error) || bump_sensor)
	{
		count++;					
		initialized = false;
		stopDrive();
	}
}

void testing_step1()
{
	
	rotate(true);
	if(!initialized)
	{
		start_theta = current_theta;
		initialized = true;
		
	}

	uv = greatest(angles::normalize_angle_positive(start_theta+1.57+angle_error), angles::normalize_angle_positive(start_theta+1.57-angle_error));
	lv = least(angles::normalize_angle_positive(start_theta+1.57+angle_error), angles::normalize_angle_positive(start_theta+1.57-angle_error));
	
	
	if((uv < current_theta && current_theta <= uv + 2*angle_error) || (lv > current_theta && current_theta > (lv - 2*angle_error)))
	{
		count++;
		initialized = false;
		stopRotate();

	}
}

void testing_step2()
{
	//("STEP 2");
	drive(true);
	if(!initialized)
	{
		start_x = current_pos_x;
		start_y = current_pos_y;
		//					//("x: %f y: %f", start_x, start_y);
		initialized = true;
	}

	dist_traveled_x = start_x - current_pos_x;
	dist_traveled_y = start_y - current_pos_y; 	
	//				//("current_x: %f current_y: %f", current_pos_x, current_pos_y);
	//				//("traveled_x: %f traveled_y: %f", dist_traveled_x, dist_traveled_y);

	dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    
	//				//("dist_traveled: %f \n", dist_traveled);

	if(bump_sensor)
	{
		stopDrive();
		initialized = false;
		count = 0;
	}

	if(test_distance-distance_error < dist_traveled && dist_traveled <= test_distance+distance_error)
	{
		moved = true;
		count++;					
		initialized = false;
		stopDrive();
	}
}

void testing_step3()
{

	rotate(false);
	if(!initialized)
	{
		start_theta = current_theta;
		initialized = true;
	}
	uv = greatest(angles::normalize_angle_positive(start_theta-1.57+angle_error), angles::normalize_angle_positive(start_theta-1.57-angle_error));
	lv = least(angles::normalize_angle_positive(start_theta-1.57+angle_error), angles::normalize_angle_positive(start_theta-1.57-angle_error));


	if((uv < current_theta && current_theta <= uv + 2*angle_error) || (lv > current_theta && current_theta > (lv - 2*angle_error)))
	{
		count++;
		initialized = false;
		stopRotate();
	}
}

void testing_step4()
{
	
	if(!initialized)
	{
		start_x = current_pos_x;
		start_y = current_pos_y;

		initialized = true;
		forward = drive_speed;
		rot = 0;

		start_theta = current_theta;
	}

	msg.linear.x = forward; //these steps are a bit different due to the possibility of having gone off an object edge, so we don't use the drive() or rotate() methods
	msg.angular.z = rot;
	dist_traveled_x = start_x - current_pos_x;
	dist_traveled_y = start_y - current_pos_y; 	


	dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    


	if(bump_sensor && !reset)
	{
		count=0;
		initialized = false;
		reset = false;
		stopRotate();

		stopDrive();
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

	//("current_theta: %f, start_theta: %f, boundaries: %f , %f", current_theta, start_theta, lv, uv);
	if((uv < current_theta && current_theta <= uv + 2*angle_error) || (lv > current_theta && current_theta > (lv - 2*angle_error)))
	{
		count++;
		initialized = false;
		reset = false;
		stopRotate();
	}
}

void testing_step5()
{
	//("STEP 5");
	drive(true);
	if(!initialized)
	{
		start_x = current_pos_x;
		start_y = current_pos_y;
		//					//("x: %f y: %f", start_x, start_y);
		initialized = true;
	}

	dist_traveled_x = start_x - current_pos_x;
	dist_traveled_y = start_y - current_pos_y; 	
	//				//("current_x: %f current_y: %f", current_pos_x, current_pos_y);
	//				//("traveled_x: %f traveled_y: %f", dist_traveled_x, dist_traveled_y);

	dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);				    
	//				//("dist_traveled: %f \n", dist_traveled);

	if(bump_sensor)
	{
		stopDrive();
		initialized = false;
		count = 0;
	}

	if(test_distance-distance_error < dist_traveled && dist_traveled <= test_distance+distance_error)
	{

		count++;					
		initialized = false;
		stopDrive();
	}
}

//bool m_line()
//{
//	uv = greatest(angles::normalize_angle_positive(goal_theta+angle_pos_error), angles::normalize_angle_positive(goal_theta-angle_pos_error));
//	lv = least(angles::normalize_angle_positive(goal_theta+angle_pos_error), angles::normalize_angle_positive(goal_theta-angle_pos_error));
//	return ();
//}

//dispatcher for the various testing routine steps, also checks to if we've regained the m-line
void tester(int step_number)
{
	if(step_number == 0)
	{
		ROS_INFO("step0");
		testing_step0();
	}

	else if(step_number ==1)
	{
		ROS_INFO("step1");
		testing_step1();			
	}

	else if(step_number == 2)
	{
		ROS_INFO("step2");
		testing_step2();
	}

	else if(step_number ==3)
	{
		ROS_INFO("step3");
		testing_step3();		
	}

	else if(step_number == 4)
	{
		ROS_INFO("step4");
		testing_step4();		
	}

	else if(step_number == 5)
	{
		ROS_INFO("step5");
		testing_step5();
	}

	//have we regained the m-line?
	uv = greatest(angles::normalize_angle_positive(goal_theta+angle_pos_error), angles::normalize_angle_positive(goal_theta-angle_pos_error));
	lv = least(angles::normalize_angle_positive(goal_theta+angle_pos_error), angles::normalize_angle_positive(goal_theta-angle_pos_error));

	current_pos_theta = atan2(current_pos_y, current_pos_x);
	current_pos_theta = angles::normalize_angle_positive(current_pos_theta);

	if (moved && fabs(goal_x)>fabs(current_pos_x) && fabs(current_pos_x)> 0 && fabs(goal_y)>fabs(current_pos_y) && fabs(current_pos_y)>0 &&
			((uv < current_pos_theta && current_pos_theta <= uv + 2*angle_pos_error) || (lv > current_pos_theta && current_pos_theta > (lv - 2*angle_pos_error))))
	{
		testing = false;
		moved = false;
		setGoalThetaFromCurrent();
	}
	

}

int main(int argc, char **argv)
{
	//Initializes ROS, and sets up a node
	ros::init(argc, argv, "create");
	ros::NodeHandle nh;

	//Creates the publisher, and tells it to publish
	//to the /cmd_vel topic, with a queue size of 100
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
	//Creates Subscribers to the odometry and contact sensor topics
	ros::Subscriber odom =nh.subscribe("odom", 100, odomCallback);
	ros::Subscriber bump =nh.subscribe("base_bumper", 100, bumpCallback);
	ros::Rate rate(10);


	setGoalThetaFromCurrent();	  


	while(ros::ok())
	{
		//safety, keeps odometry as accurate as possible (gets thrown off if the robot is driven into a wall- it doesn't move but the encoders read a change)
		if(bump_sensor)
		{
			stopDrive();
		}
		ROS_INFO("current_pos: (%f,%f), goal_pos: (%f,%f), current_theta/goal_theta: (%f/%f), current_pos_theta/goal_pos_theta: (%f/%f), moved: %d, testing: %d, bump_sensor: %d", current_pos_x,
					current_pos_y, goal_x, goal_y, current_theta, goal_theta, current_pos_theta, goal_theta, moved, testing, bump_sensor);
		if(!testing)
		{
			
			not_testing();
		}

		if(testing)
		{
			tester(count%steps);			
		}
		//safety, keeps odometry as accurate as possible (gets thrown off if the robot is driven into a wall- it doesn't move but the encoders read a change)
		if(bump_sensor)
		{
			stopDrive();
		}		

		//Publish the message
		pub.publish(msg);

		//Delays untill it is time to send another message
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

