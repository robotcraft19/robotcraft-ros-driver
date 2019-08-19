/**
 * @file robot_controller.cpp
 * @author Nicolas Filliol <nicolas.filliol@icloud.com>, Erwin Lejeune <erwin.lejeune15@gmail.com>, 
 *         Oleksandr Koreiba<alex@koreiba.com>, Jan Tiepelt, 
 *         Giovanni Alexander Bergamaschi
 * @brief Amazebot Controller Class
 * @version 0.2
 * @date 2019-08-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "square_controller.h"

SquareController::SquareController() : loop_rate(ROSRATE)
{
    // Init ros node
    this->node_handle = ros::NodeHandle();

    // Publish
    this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

    // Subscribe
    this->odom_sub = node_handle.subscribe("/odom", 10, &SquareController::odomCallback, this);
    this->ir_front_sensor_sub = node_handle.subscribe("/ir_front_sensor", 10, &SquareController::frontSensorCallback, this);
    this->ir_right_sensor_sub = node_handle.subscribe("/ir_right_sensor", 10, &SquareController::rightSensorCallback, this);
    this->ir_left_sensor_sub = node_handle.subscribe("/ir_left_sensor", 10, &SquareController::leftSensorCallback, this);

    InitPose.x = InitPose.y = InitPose.theta = 0.0;
    rotation = false;
}

float SquareController::degToRad(int angle)
{
    float rad = angle * M_PI / 180;
    return (rad);
}

int SquareController::radToDeg(float angle)
{
    float deg = angle * 180 / M_PI;
    return (deg);
}

float SquareController::calcDistance(float x1, float x2, float y1, float y2)
{
    return (sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)));
}

geometry_msgs::Twist SquareController::calculateCommand(float squareSize)
{
    auto msg = geometry_msgs::Twist();
    float distance = calcDistance(InitPose.x, poseRobot.x, InitPose.y, poseRobot.y);
    float currentAngle = q.getAngle();

    // Shittiest fucking way to code this ffs this terrible Imma go straight back to the other one cause damn this is a blasphemy
    // PS : I think it doesnt work because I publish IN the function and not in calculateCommand that returns msg
    // MIGHT JUST FIX IT OLOLO
    switch(rotation) 
    {
        case(0): // No rotation
            if(distance < squareSize) 
            {
                msg.linear.x = 1.0;
                msg.angular.z = 0.0;
            } 
            else 
            {
                msg.linear.x = 0.0;
                rotation = 1;
                InitPose.theta = currentAngle;
            }
            break;

        case(1): // Calculate Rotation
            if (InitPose.theta < (M_PI/2))
            {
                target_angle1 = InitPose.theta + (M_PI/2);
                target_angle2 = -InitPose.theta + (M_PI/2);
            }
            else
            {
                target_angle1 = M_PI - ((M_PI/2) - (M_PI - InitPose.theta));
                target_angle2 = InitPose.theta - (M_PI/2);
            }
            rotation = 2;
            break;
        
        case(2): // Rotate
            if (rotation == 2 && (abs(currentAngle - target_angle2) > 0.1 && abs(currentAngle - target_angle1) > 0.1))
            {
                msg.angular.z = -0.60;
                msg.linear.x = 0.0;
            }
            else 
            {
                msg.angular.z = 0.0;
                rotation = 0;
                InitPose.y = poseRobot.y;
                InitPose.x = poseRobot.x;
            }
            break;

        default: 
            ROS_ERROR("You should not reach that statement !!!");
    }

    return msg;
}

void SquareController::frontSensorCallback(const sensor_msgs::Range& msg)
{
    frontIR = msg.range;
    if (frontIR < 0.15)
        ROS_WARN("Warning : Front sensor detecting obstacle at %f meters", frontIR);
    if (frontIR < 0.10)
        ROS_ERROR("You should not reach that statement : front sensor detecting obstacle closer than 0.1 meters !");
}

void SquareController::rightSensorCallback(const sensor_msgs::Range& msg)
{
    rightIR = msg.range;
    if (rightIR < 0.15)
        ROS_WARN("Warning : Right sensor detecting obstacle at %f meters", rightIR);
    if (rightIR < 0.10)
        ROS_ERROR("You should not reach that statement : right sensor detecting obstacle closer than 0.1 meters !");
}

void SquareController::leftSensorCallback(const sensor_msgs::Range& msg)
{
    leftIR = msg.range;
    if (leftIR < 0.15)
        ROS_WARN("Warning : Left sensor detecting obstacle at %f meters", leftIR);
    if (leftIR < 0.10)
        ROS_ERROR("You should not reach that statement : left sensor detecting obstacle closer than 0.1 meters !");
}

void SquareController::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    poseRobot.x = msg->pose.pose.position.x;
    poseRobot.y = msg->pose.pose.position.y;
    poseRobot.theta = msg->pose.pose.orientation.z;
    tf2::convert(msg->pose.pose.orientation, q);
}

void SquareController::run()
{
    while (ros::ok())
    {
        // Calculate the command to apply
        auto msg = calculateCommand(0.8);

        cmd_vel_pub.publish(msg);

        ros::spinOnce();

        // And throttle the loop
        loop_rate.sleep();
    }
}