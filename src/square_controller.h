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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"


#define ROSRATE            10

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

	ros::Publisher cmd_vel_pub;

    ros::Subscriber front_distance_sub;
    ros::Subscriber left_distance_sub;
    ros::Subscriber right_distance_sub;

    // Message initialization
    geometry_msgs::Twist twist_msg;	

    Pose poseRobot;
    Velocity velocityRobot;

    // Sensor Data
    float left_distance;
    float front_distance;
    float right_distance;

    float leftIR, frontIR, rightIR;

    void drawSquare();

    void frontDistanceCallback(const std_msgs::Float32& front_distance_msgs);
    void rightDistanceCallback(const std_msgs::Float32& right_distance_msgs);
    void leftDistanceCallback(const std_msgs::Float32& left_distance_msgs);
public:

    AmazebotController();
    void moveForward(float speed);
    void moveBackwards(float speed);
    void turnLeft(float angle);
    void turnRight(float angle);
    void stopRobot();
    void run();
};

#endif /** AMAZEBOT_CONTROLLER_H **/
