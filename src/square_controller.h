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
#include <math.h>
#include <ros/console.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Odometry.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/Range.h"

#define ROSRATE    10

typedef struct {
    float x, y, theta;
} Pose;

class SquareController
{

    private:
        ros::NodeHandle node_handle;

        ros::Rate loop_rate;


        ros::Publisher cmd_vel_pub;
        ros::Subscriber ir_front_sensor_sub;
        ros::Subscriber ir_left_sensor_sub;
        ros::Subscriber ir_right_sensor_sub;
        ros::Subscriber odom_sub;

        geometry_msgs::Pose2D poseRobot;

        Pose InitPose;

        float target_angle1, target_angle2;
   
   
        float leftIR, frontIR, rightIR;

        tf2::Quaternion q;
        int rotation;
        geometry_msgs::Twist calculateCommand(float squareSize);
        
        void frontSensorCallback(const sensor_msgs::Range& msg);
        void rightSensorCallback(const sensor_msgs::Range& msg);
        void leftSensorCallback(const sensor_msgs::Range& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

        float degToRad(int angle);
        int radToDeg(float angle);
        float calcDistance(float x1, float x2, float y1, float y2);

    public:
        SquareController();
        void run();

};




#endif /** AMAZEBOT_CONTROLLER_H **/
