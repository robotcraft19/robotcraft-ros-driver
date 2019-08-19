/**
 * @file robot_controller.h
 * @author Nicolas Filliol <nicolas.filliol@icloud.com>, Erwin Lejeune <erwin.lejeune15@gmail.com>, 
 *         Oleksandr Koreiba <alex@koreiba.com>, Jan Tiepelt <>, 
 *         Giovanni Alexander Bergamaschi <>
 * @brief Amazebot Controller Class
 * @version 0.2
 * @date 2019-08-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef AMAZEBOT_CONTROLLER_H
#define AMAZEBOT_CONTROLLER_H

#include <iostream>
#include <cstdlib>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define THRESHOLD_DISTANCE 0.15
#define ROSRATE            10
#define ROSPERIOD          1/(ROSRATE/4)

typedef struct {
    float x, y, theta;
} Pose;

typedef struct {
    float x, y, theta;
} Velocity;

class AmazebotController {

private:
    ros::NodeHandle node_handle;

    ros::Rate loop_rate;
    
    //Subscribers
	ros::Subscriber odom_sub;
	ros::Subscriber front_distance_sub;
	ros::Subscriber right_distance_sub;
	ros::Subscriber left_distance_sub;

	//Publishers
    ros::Publisher cmd_vel_pub;
	//ros::Publisher odom_pub;

    // Message initialization
    nav_msgs::Odometry odom_msg;
	sensor_msgs::Range ir_front_msg, ir_left_msg, ir_right_msg;
	geometry_msgs::Pose2D pose_msg;	

    
    // Transform Helpers
    tf2::Quaternion q;
    Pose initialPoseRobot;
    Pose poseRobot;
    Velocity velocityRobot;

    // Sensor Data
    float left_distance;
    float front_distance;
    float right_distance;

    float leftIR, frontIR, rightIR;

    int Led1_R, Led1_G, Led1_B;
    int Led2_R, Led2_G, Led2_B;

    void drawSquare();


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void frontDistanceCallback(const std_msgs::Float32& front_distance_msgs);
    void rightDistanceCallback(const std_msgs::Float32& right_distance_msgs);
    void leftDistanceCallback(const std_msgs::Float32& left_distance_msgs);

    void publish();
    void subscribe();

public:

    AmazebotController();
    float degToRad(int angle);
    int radToDeg(float angle);
    float calcDistance(float x1, float x2, float y1, float y2);
    void moveForward(float distance, float speed);
    void moveBackwards(float distance, float speed);
    void turnLeft(int angle, float speed);
    void turnRight(int angle, float speed);
    void stopRobot();
    void run();
};

#endif /** AMAZEBOT_CONTROLLER_H **/
