/**
 * @file square_test.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-08-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

typedef struct {
    float x, y, theta;
} Pose;

typedef struct {
    float x, y, theta;
} Velocity;

Pose robotPose;
Velocity robotVelocity;
ros::Time current_time, last_time;
nav_msgs::Odometry odom_msg;

void go_straight()
{	
 	odom_msg.twist.twist.linear.x =2.0;
	odom_msg.twist.twist.angular.z=0.0;
	
}

void turn()
{
	odom_msg.twist.twist.linear.x =0.0;
 	odom_msg.twist.twist.angular.z=1.57;
}

void stop()
{

	odom_msg.twist.twist.linear.x =0.0;
 	odom_msg.twist.twist.angular.z=0.0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "square_test");
	ros::NodeHandle node_handle;
  	ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>("/odom",10 );
 	while (ros::ok())
  	{
	go_straight();
	odom_pub.publish(odom_msg);
	ros::Duration(5.0);
	turn();
	odom_pub.publish(odom_msg);
	ros::Duration(5.0);
	stop();
	odom_pub.publish(odom_msg);
	ros::Duration(5.0);
	}
    ros::spinOnce();
	return 0;
  }