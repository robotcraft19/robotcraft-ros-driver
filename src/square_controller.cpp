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

float AmazebotController::degToRad(int angle)
{
    float rad = angle * M_PI/180;
    return(rad);
}

int AmazebotController::radToDeg(float angle)
{
    float deg = angle * 180/M_PI;
    return(deg);
}

float AmazebotController::calcDistance(float x1, float x2, float y1, float y2)
{
    return(sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))) * 100;
}

void AmazebotController::stopRobot()
{
    auto msg = geometry_msgs::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub.publish(msg);
}

void AmazebotController::moveForward(float distance, float speed)
{
    auto msg = geometry_msgs::Twist();
    msg.linear.x = speed;
    msg.angular.z = 0.0;
    std::size_t rosloop = 0;
    float deltaT = distance/speed;
    while (rosloop < (deltaT/ROSPERIOD))
    {
        cmd_vel_pub.publish(msg);
        this->loop_rate.sleep();
        ++rosloop;
    }
    this->stopRobot();
}

void AmazebotController::moveBackwards(float distance, float speed)
{
    auto msg = geometry_msgs::Twist();
    msg.linear.x = -speed;
    msg.angular.z = 0.0;
    std::size_t rosloop = 0;
    float deltaT = distance/speed;
    while (rosloop < (deltaT/ROSPERIOD))
    {
        cmd_vel_pub.publish(msg);
        this->loop_rate.sleep();
        ++rosloop;
    }
    this->stopRobot();
}

void AmazebotController::turnLeft(int angle, float speed)
{
    auto msg = geometry_msgs::Twist();
    msg.angular.z = speed;
    msg.linear.x = 0.0;
    std::size_t rosloop = 0;
    float deltaT = degToRad(angle)/speed;
    while (rosloop < (deltaT/ROSPERIOD))
    {
        cmd_vel_pub.publish(msg);
        this->loop_rate.sleep();
        ++rosloop;
    }
    this->stopRobot();

}

void AmazebotController::turnRight(int angle, float speed)
{
    auto msg = geometry_msgs::Twist();
    msg.angular.z = -speed;
    msg.linear.x = 0.0;
    std::size_t rosloop = 0;
    float deltaT = degToRad(angle)/speed;
    while (rosloop < (deltaT/ROSPERIOD))
    {
        cmd_vel_pub.publish(msg);
        this->loop_rate.sleep();
        ++rosloop;
    }
    this->stopRobot();
}

/**
 * @brief Calculates controller commands and returns message of type Twist
 * 
 * @return geometry_msgs::Twist including linear and angular velocity
 */
void AmazebotController::drawSquare() 
{   
    moveForward(5, 0.3);
    loop_rate.sleep();
    turnLeft(180, (M_PI/2));
    loop_rate.sleep();
    
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
 * @brief 
 * 
 */
void AmazebotController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    // Publish pose data
	poseRobot.x = msg->pose.pose.position.x;
    poseRobot.y = msg->pose.pose.position.y;
    poseRobot.theta = msg->pose.pose.orientation.z;
    tf2::convert(msg->pose.pose.orientation,q);
}

/**
 * @brief Construct a new Robot Controller:: Robot Controller object
 * 
 */
AmazebotController::AmazebotController() : loop_rate(ROSRATE)
{
    // Initialize ROS
    this->node_handle = ros::NodeHandle();

    Led1_R = Led2_R = 0;
    Led1_G = Led2_G = 150;
    Led1_B = Led2_B = 100;

    initialPoseRobot.x = initialPoseRobot.y = initialPoseRobot.theta = 0;

    //Subscribers
	this->odom_sub = node_handle.subscribe("/odom", 10, &AmazebotController::odomCallback, this);
	this->front_distance_sub = node_handle.subscribe("/front_distance", 10, &AmazebotController::frontDistanceCallback, this);
	this->right_distance_sub = node_handle.subscribe("/right_distance", 10, &AmazebotController::rightDistanceCallback, this);
	this->left_distance_sub = node_handle.subscribe("/left_distance", 10, &AmazebotController::leftDistanceCallback, this);

	//Publishers
    this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

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
        drawSquare();

        ros::spinOnce();

        // And throttle the this->loop
        this->loop_rate.sleep();
    }
}
