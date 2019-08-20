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

#include "amazebot_controller.h"

/**
 * @brief 
 * 
 * @param angle 
 * @return float 
 */
float AmazebotController::degToRad(int angle)
{
    float rad = angle * M_PI/180;
    return(rad);
}

/**
 * @brief 
 * 
 * @param angle 
 * @return int 
 */
int AmazebotController::radToDeg(float angle)
{
    float deg = angle * 180/M_PI;
    return(deg);
}

/**
 * @brief 
 * 
 * @param x1 
 * @param x2 
 * @param y1 
 * @param y2 
 * @return float 
 */
float AmazebotController::calcDistance(float x1, float x2, float y1, float y2)
{
    return(sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))) * 100;
}

/**
 * @brief stops the robot
 * 
 */
void AmazebotController::stopRobot()
{
    auto msg = geometry_msgs::Twist();
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub.publish(msg);
}

/**
 * @brief Calculates controller commands and returns message of type Twist
 * 
 * @return geometry_msgs::Twist including linear and angular velocity
 */
geometry_msgs::Twist AmazebotController::calculateCommand() 
{   
    calculateRobotLost();

    auto msg = geometry_msgs::Twist();
        
    if (frontIR < THRESHOLD_DISTANCE) 
    {
        // Prevent robot from crashing
        msg.angular.z = 1.0;
        msg.linear.x = -0.05;
    } 
    else if (robot_lost == true)
    {
        // Robot is lost, go straight to find wall
        msg.linear.x = 0.5;
    } 
    else 
    {
        // Robot keeps using normal PID controller
        float gain = calculateGain(rightIR);
        msg.linear.x = 0.5;
        msg.angular.z = gain;
    }
    return msg;
}

/**
 * @brief 
 * 
 * @param front_distance_msgs 
 */
void AmazebotController::frontDistanceCallback(const std_msgs::Float32& front_distance_msgs)
{
	frontIR = front_distance_msgs.data;
}

/**
 * @brief 
 * 
 * @param left_distance_msgs 
 */
void AmazebotController::leftDistanceCallback(const std_msgs::Float32& left_distance_msgs)
{
	leftIR = left_distance_msgs.data;
}

/**
 * @brief 
 * 
 * @param right_distance_msgs 
 */
void AmazebotController::rightDistanceCallback(const std_msgs::Float32& right_distance_msgs)
{
	rightIR = right_distance_msgs.data;
}

/**
 * @brief 
 * 
 * @param Pose2D_msgs 
 */
void AmazebotController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    poseRobot.x = msg->pose.pose.position.x;
	poseRobot.y = msg->pose.pose.position.y;
	poseRobot.theta = msg->pose.pose.orientation.z;
    tf2::convert(msg->pose.pose.orientation,q);
}

/**
 * @brief Laser Scan callback of subscriber
 * 
 * @param msg LaserScan message received by subscriber
 */
void AmazebotController::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    // Calculate smallest distance
    obstacle_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    ROS_INFO("Min distance to obstacle: %f", obstacle_distance);
}

/**
 * @brief PID controller that calculates angular rotation
 * 
 * @param value Current distance to wall (real value) of robot
 * @return float Outputs gain (angular velocity)
 */
float AmazebotController::calculateGain(float value) 
{
    float error = this->target_value - value;
    float new_der_err = error - this->old_prop_error;
    float new_int_err = this->integral_error + error;

    float gain = this->KP*error + this->KI*new_int_err*this->time_interval
                 + this->KD*new_der_err/this->time_interval;

    this->old_prop_error = error;
    this->integral_error = new_int_err;         

    return gain;
}

/**
 * @brief Calculates and checks if the robot is lost
 * 
 */
void AmazebotController::calculateRobotLost() 
{
    // Calculations needed to check if robot is lost
    if (frontIR > THRESHOLD_DISTANCE && rightIR > THRESHOLD_DISTANCE 
        && leftIR > THRESHOLD_DISTANCE) 
    {
            ++lost_counter;

            if (lost_counter >= 75) robot_lost = true;
    } 
    else if(frontIR < THRESHOLD_DISTANCE || rightIR < THRESHOLD_DISTANCE) 
    {
            robot_lost = false;
            lost_counter = 0;
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

    Led1_R = Led2_R = 0;
    Led1_G = Led2_G = 150;
    Led1_B = Led2_B = 100;

    initialPoseRobot.x = initialPoseRobot.y = initialPoseRobot.theta = 0;
    velocityRobot.x = 0.0;
    velocityRobot.y = 0.0;
    velocityRobot.theta = 0.0;

    //Subscribers
    this->laser_sub = node_handle.subscribe("/base_scan", 10, &AmazebotController::laserCallback, this);
	this->odom_sub = node_handle.subscribe("/odom", 10, &AmazebotController::odomCallback, this);
	this->front_distance_sub = node_handle.subscribe("/ir_front_distance", 10, &AmazebotController::frontDistanceCallback, this);
	this->right_distance_sub = node_handle.subscribe("/ir_right_distance", 10, &AmazebotController::rightDistanceCallback, this);
	this->left_distance_sub = node_handle.subscribe("/ir_left_distance", 10, &AmazebotController::leftDistanceCallback, this);

	//Publishers
    this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    this->rgb_leds_pub = node_handle.advertise<std_msgs::UInt8MultiArray>("/rgb_leds", 60);
	this->initial_pose_pub = node_handle.advertise<geometry_msgs::Pose2D>("/initial_pose", 10);
	this->ir_front_pub = node_handle.advertise<sensor_msgs::Range>("/ir_front_sensor",10);
	this->ir_left_pub  = node_handle.advertise<sensor_msgs::Range>("/ir_left_sensor",10);
    this->ir_right_pub = node_handle.advertise<sensor_msgs::Range>("/ir_right_sensor",10);

}

/**
 * @brief Run ROS
 * 
 */
void AmazebotController::run()
{
    current_time = ros::Time::now();
  	last_time = ros::Time::now();
    // Send messages in a this->loop
    while (ros::ok()) 
    {
        current_time = ros::Time::now();	\
        // Calculate the command to apply
        auto msg = calculateCommand();

        // Publish the new command
        this->cmd_vel_pub.publish(msg);

        last_time = current_time;
        ros::spinOnce();

        // And throttle the this->loop
        this->loop_rate.sleep();
    }
}
