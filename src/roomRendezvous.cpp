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
#include <ctime>
#include <cstdlib>

//holds the current position of the robot, its current orientation
double current_pos_x;
double current_pos_y;
double current_theta;
//bug2 algorithm needs to store a line from the origin to the goal position, so the angle of the line the robot is currently on is recorded here
double current_pos_theta;




//used when the robot is spinning in place to describe the desired end orientation
double goal_theta;

//how fast the robot moves
double rot_vel=.15;
double drive_speed = .1;

//allowable errors to account for the robot coasting a bit after being sent one stop command
double angle_error = .005;
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



double robot_radius = 0.16495;



//temporarily holds rotational speed and forward speed during some operations
double rot;
double forward;

//holds the calculated upper and lower error bounds for a specific motion
double uv;
double lv;

//the number of steps in the testing routine, used to loop back through them after finishing one test cycle
int count = 0;
int steps = 2;

int count1 = 0;
int steps1 = 3;

int count2 = 0;
int steps2 = 2;

int count3 = 0;
int steps3 = 2;

int rando;


bool initialized = false;
bool drive_initialized = false;
bool has_traveled = false;
bool rot_initialized = false;
bool rot_half = false;
bool bump_sensor = false;
bool testing = false;
bool reset = false;
bool moved = false;
bool explored = false;

double r = 1.225;
int radius = 1;
int i = 1;
int n = 0;
int j = 0;

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

void stopDrive()
{
	msg.linear.x = 0;
}

void stopRotate()
{
	msg.angular.z = 0;
}

void drive(bool x, double distance, int counter) //true for forward, false for backwards
{
	if(x)
	{
		msg.linear.x = drive_speed;
	
	}
	else
	{
		msg.linear.x = -drive_speed;
	
	}

	if(!drive_initialized)
	{
		start_x = current_pos_x;
		start_y = current_pos_y;
		drive_initialized = true;
	}
	
	if(dist_traveled > distance_error)
	{
		has_traveled = true;
	}

	dist_traveled_x = start_x - current_pos_x;
	dist_traveled_y = start_y - current_pos_y; 	
	dist_traveled = sqrt(dist_traveled_x*dist_traveled_x + dist_traveled_y*dist_traveled_y);	

	ROS_INFO("distance: %f, distance-distance_error: %f, distance+distance_error: %f, dist_traveled: %f", distance, distance-distance_error, distance+distance_error, dist_traveled);

	if((distance-distance_error < dist_traveled && dist_traveled < distance+distance_error) || (-distance_error < dist_traveled && dist_traveled < distance_error && has_traveled)) //this doesn't work.
	{
		if (counter == 0)
		{
			count++;
		}
		else if(counter == 1)
		{
			count1++;
		}
		else if(counter ==2)
		{
			count2++;
		}
		else if(counter == 3)
		{
			count3++;
		}
		else if(counter == 4)
		{
			count3++;
			count++;
		}
		drive_initialized = false;
		has_traveled = false;
		stopDrive();
	}

}

void rotate(bool x, double radians, int counter) //true for counterclockwise, false for clockwise; counter indicates which variable to increment at the end to move onto the next cycle step 
//counter=0-->overall program flow
{
	if (x)
	{
		
		msg.angular.z = rot_vel;
		uv = greatest(angles::normalize_angle_positive(start_theta+radians+angle_error), angles::normalize_angle_positive(start_theta+radians-angle_error));
		lv = least(angles::normalize_angle_positive(start_theta+radians+angle_error), angles::normalize_angle_positive(start_theta+radians-angle_error));
	}
	else
	{
		
		msg.angular.z = -rot_vel;
		uv = greatest(angles::normalize_angle_positive(start_theta-radians+angle_error), angles::normalize_angle_positive(start_theta-radians-angle_error));
		lv = least(angles::normalize_angle_positive(start_theta-radians+angle_error), angles::normalize_angle_positive(start_theta-radians-angle_error));
	}
	if(!rot_initialized)
	{
		start_theta = current_theta;

		rot_initialized = true;
	}
	
	if(current_theta-start_theta > (uv-start_theta)/2)
	{
		rot_half = true;
	}


	ROS_INFO("uv: %f, lv: %f, current_theta: %f", uv, lv, current_theta);

	if(((uv < current_theta && current_theta <= uv + 2*angle_error) || (lv > current_theta && current_theta > (lv - 2*angle_error))) && rot_half)
	{
		if(counter == 0)
		{
			count++;
		}
		else if(counter == 1)
		{
			count1++;
		}
		else if(counter ==2)
		{
			count2++;
		}
		else if(counter == 3)
		{
			count3++;
		}
		else if(counter == 4)
		{
			count3++;
			count++;
		}
		rot_initialized = false;
		rot_half = false;
		stopRotate();
	}
}

void circle(int size)
{
	if(count3%steps3 ==0)
	{
		rotate(false, 1.57, 3);
	}
	else if(count3%steps3 ==1)
	{			
		rotate(false, 6.28, 4);
		drive(true, 6.28*(2*j +1)*radius,4);
	}
}

void wait()
{
	if(count2%steps2 == 0)
	{
		drive((2*n+1)*radius + 6.28*(n+1)*(n+1)*radius, true, 2);
	}
	else if(count2%steps2==1)
	{
		drive((2*n+1)*radius + 6.28*(n+1)*(n+1)*radius, false, 0);
	}
}

void move()
{
	if(j <= n)
	{
		if (count1%steps1 == 0)
		{
			if(j == 0)
			{
				drive(true, radius, 1);
			}
			else
			{
				drive(true, 2*radius, 1);
			}
		}
		else if(count1%steps1 == 1)
		{
			circle((2*j+1)*radius);
		}
		else if(count1%steps1 == 2)
		{
			j++;
			count1++;
		}
	}
	else
	{
		drive(false, 2*n*radius + radius, 0);
	}
}

void ssrS(int decision)
{
	if(!initialized)
	{
		rando = rand()%2;
		initialized = true;
	}
	n = floor((pow(r,i))/(2*radius));
	if(count%steps == 0)
	{
		if(decision ==1)
		{
			wait();
		}
		if(decision == 0)
		{
			move();
		}
	}
	if(count%steps == 1)
	{
		i++;
		initialized = false;
		count++;
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

	srand(time(0));
	rando = rand()%2;

	while(ros::ok())
	{
		if(bump_sensor)
		{
			stopDrive();
			stopRotate();
		}
		else
		{
			ssrS(rando);
		}
		
		ROS_INFO("rando: %d", rando);
		

		//Publish the message
		pub.publish(msg);

		//Delays untill it is time to send another message
		rate.sleep();
		ros::spinOnce();

	}
	return 0;
}