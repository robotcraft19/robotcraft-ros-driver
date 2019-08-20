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
#define ROSPERIOD          1/ROSRATE

typedef struct {
    float x, y, theta;
} Pose;

typedef struct {
    float x, y, theta;
} Velocity;

class AmazebotController {

private:
    ros::NodeHandle node_handle;
    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate loop_rate;
    
    //Subscribers
    ros::Subscriber laser_sub;
	ros::Subscriber odom_sub;
	ros::Subscriber front_distance_sub;
	ros::Subscriber right_distance_sub;
	ros::Subscriber left_distance_sub;

	//Publishers
    ros::Publisher cmd_vel_pub;
    ros::Publisher rgb_leds_pub;
	ros::Publisher initial_pose_pub;
    ros::Publisher ir_front_pub;
    ros::Publisher ir_right_pub;
    ros::Publisher ir_left_pub;

    // Message initialization
    nav_msgs::Odometry odom_msg;
	sensor_msgs::Range ir_front_msg, ir_left_msg, ir_right_msg;
    std_msgs::UInt8MultiArray rgb_leds_msg;
	geometry_msgs::Pose2D initial_pose_msg;	
	ros::Time current_time, last_time;

    
    // Transform Helpers
    Pose initialPoseRobot;
    Pose poseRobot;
    Velocity velocityRobot;
    tf2::Quaternion q;

    // Sensor Data
    float obstacle_distance;

    float leftIR, frontIR, rightIR;

    int Led1_R, Led1_G, Led1_B;
    int Led2_R, Led2_G, Led2_B;

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
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void frontDistanceCallback(const std_msgs::Float32& front_distance_msgs);
    void rightDistanceCallback(const std_msgs::Float32& right_distance_msgs);
    void leftDistanceCallback(const std_msgs::Float32& left_distance_msgs);

    //void odometryHelper();
    void sensorHelper();
    void initialPose();

    float calculateGain(float value);
    void calculateRobotLost();

public:

    AmazebotController();
    float degToRad(int angle);
    int radToDeg(float angle);
    float calcDistance(float x1, float x2, float y1, float y2);
    void stopRobot();
    void run();
};

#endif /** AMAZEBOT_CONTROLLER_H **/
