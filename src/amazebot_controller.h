/**
 * @file robot_controller.h
 * @author Nicolas Filliol <nicolas.filliol@icloud.com>, Erwin Lejeune <erwin.lejeune15@gmail.com>, 
 *         Oleksandr Koreiba <>, Jan Tiepelt <>, 
 *         Giovanni Alexander Bergamaschi <>
 * @brief Amazebot Controller Class
 * @version 0.1
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

#define THRESHOLD_DISTANCE 0.3


class AmazebotController {

private:

    ros::NodeHandle node_handle;
    ros::Publisher cmd_vel_pub;
    
    //Subscribers
	ros::Subscriber pose_sub;
	ros::Subscriber front_distance_sub;
	ros::Subscriber right_distance_sub;
	ros::Subscriber left_distance_sub;


	//Publishers
	ros::Publisher odom_pub;
    ros::Publisher rgb_leds_pub;
	ros::Publisher reset_pose_pub;


    // Message initialization
    nav_msgs::Odometry odom_msg;
	sensor_msgs::Range ir_front_msg, ir_left_msg, ir_right_msg;
    std_msgs::UInt8MultiArray rgb_leds_msg;
	geometry_msgs::Pose2D reset_pose_msg;	
	ros::Time current_time, last_time;


    float left_distance;
    float front_distance;
    float right_distance;

    float leftIR, frontIR, rightIR;

    // PID control
    float old_prop_error;
    float integral_error;
    
    float target_value = THRESHOLD_DISTANCE;
    float KP = 10.0;
    float KI = 0.0;
    float KD = 0.0;
    float time_interval = 0.1;

    bool robot_lost;
    int lost_counter;

    geometry_msgs::Twist calculateCommand();


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void poseCallback(const geometry_msgs::Pose2D& Pose2D_msgs);
    void front_distance_callback(const std_msgs::Float32& front_distance_msgs);
    void left_distance_callback(const std_msgs::Float32& left_distance_msgs);
    void right_distance_callback(const std_msgs::Float32& right_distance_msgs);

    float calculateGain(float value);
    void calculateRobotLost();

public:

    AmazebotController();
    void run();

};

#endif /** AMAZEBOT_CONTROLLER_H **/