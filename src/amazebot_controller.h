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

#define THRESHOLD_DISTANCE 0.20

typedef struct {
    float x, y, theta;
} Pose;

typedef struct {
    float x, y, theta;
} Velocity;

class AmazebotController {

private:
    ros::NodeHandle node_handle;
    
    //Subscribers
	ros::Subscriber odom_sub;
	ros::Subscriber front_distance_sub;
	ros::Subscriber right_distance_sub;
	ros::Subscriber left_distance_sub;

	//Publishers
    ros::Publisher cmd_vel_pub;
    ros::Publisher rgb_leds_pub;
	ros::Publisher set_pose_pub;
    
    // Transform Helpers
    Pose poseRobot;
    tf2::Quaternion q;

    // Sensor Data
    float leftIR, frontIR, rightIR;

    // PID control
    float old_prop_error;
    float integral_error;
    
    float target_value = THRESHOLD_DISTANCE;
    float KP = 10.00;
    float KI = 0.00;
    float KD = 0.00;
    float time_interval = 0.1;

    bool robot_lost;
    int lost_counter;

    geometry_msgs::Twist calculateCommand();


    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void frontDistanceCallback(const sensor_msgs::Range& front_distance_msgs);
    void rightDistanceCallback(const sensor_msgs::Range& right_distance_msgs);
    void leftDistanceCallback(const sensor_msgs::Range& left_distance_msgs);

    float calculateGain(float value);
    void calculateRobotLost();

public:

    AmazebotController();
    void run();
};

#endif /** AMAZEBOT_CONTROLLER_H **/
