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

void AmazebotController::stopRobot()
{
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
}

void AmazebotController::moveForward(float speed)
{
    twist_msg.linear.x = speed;
	twist_msg.angular.z= 0.0;
}

void AmazebotController::moveBackwards(float speed)
{
    twist_msg.linear.x = -speed;
	twist_msg.angular.z=0.0;
}

void AmazebotController::turnLeft(float angle)
{
    twist_msg.linear.x = 0.0;
 	twist_msg.angular.z= angle;
}

void AmazebotController::turnRight(float angle)
{
    twist_msg.linear.x = 0.0;
 	twist_msg.angular.z = -angle;
}

/**
 * @brief Calculates controller commands and returns message of type Twist
 * 
 * @return geometry_msgs::Twist including linear and angular velocity
 */
void AmazebotController::drawSquare() 
{   
    moveForward(2.0);
    cmd_vel_pub.publish(twist_msg);
    ros::Duration(5.0);
    turnLeft(M_PI/2);
    cmd_vel_pub.publish(twist_msg);
    ros::Duration(5.0);
    stopRobot();
    cmd_vel_pub.publish(twist_msg);
    ros::Duration(5.0);
}

/**
 * @brief 
 * 
 * @param front_distance_msgs 
 */
void AmazebotController::frontDistanceCallback(const std_msgs::Float32& front_distance_msgs)
{
	frontIR = front_distance_msgs.data;
    if (frontIR < 0.15) 
    {
        ROS_WARN("Careful ! Obstacle at detected at %f meters by front sensor", frontIR);
    }
}

/**
 * @brief 
 * 
 * @param left_distance_msgs 
 */
void AmazebotController::leftDistanceCallback(const std_msgs::Float32& left_distance_msgs)
{
	leftIR = left_distance_msgs.data;
    if (leftIR < 0.15) 
    {
        ROS_WARN("Careful ! Obstacle at detected at %f meters by left sensor", leftIR);
    }
}

/**
 * @brief 
 * 
 * @param right_distance_msgs 
 */
void AmazebotController::rightDistanceCallback(const std_msgs::Float32& right_distance_msgs)
{
	rightIR = right_distance_msgs.data;
    if (rightIR < 0.15) 
    {
        ROS_WARN("Careful ! Obstacle at detected at %f meters by right sensor", rightIR);
    }
}

/**
 * @brief Construct a new Robot Controller:: Robot Controller object
 * 
 */
AmazebotController::AmazebotController() : loop_rate(ROSRATE)
{
    // Initialize ROS
    this->node_handle = ros::NodeHandle();

    //Subscribers
	this->front_distance_sub = node_handle.subscribe("/front_distance", 10, &AmazebotController::frontDistanceCallback, this);
	this->right_distance_sub = node_handle.subscribe("/right_distance", 10, &AmazebotController::rightDistanceCallback, this);
	this->left_distance_sub = node_handle.subscribe("/left_distance", 10, &AmazebotController::leftDistanceCallback, this);

	//Publishers
    this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("/cmd_vel",10 );

}

/**
 * @brief Run ROS
 * 
 */
void AmazebotController::run()
{
    while (ros::ok()) 
    {
        // Calculate the command to apply
        moveForward(2.0);
        cmd_vel_pub.publish(twist_msg);
        ros::Duration(5.0);
        turnLeft(M_PI/2);
        cmd_vel_pub.publish(twist_msg);
        ros::Duration(5.0);
        stopRobot();
        cmd_vel_pub.publish(twist_msg);
        ros::Duration(5.0);

        ros::spinOnce();

        // And throttle the this->loop
        this->loop_rate.sleep();
    }
}
