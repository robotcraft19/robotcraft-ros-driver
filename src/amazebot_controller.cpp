/**
 * @file robot_controller.cpp
 * @author Nicolas Filliol <nicolas.filliol@icloud.com>, Erwin Lejeune <erwin.lejeune15@gmail.com>, 
 *         Oleksandr Koreiba, Jan Tiepelt, 
 *         Giovanni Alexander Bergamaschi
 * @brief Amazebot Controller Class
 * @version 0.1
 * @date 2019-08-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "amazebot_controller.h"

/**
 * @brief Calculates controller commands and returns message of type Twist
 * 
 * @return geometry_msgs::Twist including linear and angular velocity
 */
geometry_msgs::Twist AmazebotController::calculateCommand() 
{   
    calculateRobotLost();

    auto msg = geometry_msgs::Twist();
        
    if (front_distance < THRESHOLD_DISTANCE) 
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
        float gain = calculateGain(right_distance);
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
void AmazebotController::front_distance_callback(const std_msgs::Float32& front_distance_msgs)
{
	frontIR = front_distance_msgs.data;
}

/**
 * @brief 
 * 
 * @param left_distance_msgs 
 */
void AmazebotController::left_distance_callback(const std_msgs::Float32& left_distance_msgs)
{
	leftIR = left_distance_msgs.data;
}

/**
 * @brief 
 * 
 * @param right_distance_msgs 
 */
void AmazebotController::right_distance_callback(const std_msgs::Float32& right_distance_msgs)
{
	rightIR = right_distance_msgs.data;
}


/**
 * @brief Laser Scan callback of subscriber
 * 
 * @param msg LaserScan message received by subscriber
 */
void AmazebotController::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    // Calculate array size of ranges
    int ranges_len = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    int split_size = ranges_len / 3;

    // Split sensor data into three areas and extract smallest distance
    right_distance = *std::min_element(msg->ranges.begin(), msg->ranges.begin()+split_size);
    front_distance = *std::min_element(msg->ranges.begin()+split_size, msg->ranges.begin()+2*split_size);
    left_distance = *std::min_element(msg->ranges.begin()+2*split_size, msg->ranges.begin()+ranges_len);
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
    if (front_distance > THRESHOLD_DISTANCE && right_distance > THRESHOLD_DISTANCE 
        && left_distance > THRESHOLD_DISTANCE) 
    {
            ++lost_counter;

            if (lost_counter >= 75) robot_lost = true;
    } 
    else if(front_distance < THRESHOLD_DISTANCE || right_distance < THRESHOLD_DISTANCE) 
    {
            robot_lost = false;
            lost_counter = 0;
    }
}

/**
 * @brief Construct a new Robot Controller:: Robot Controller object
 * 
 */
AmazebotController::AmazebotController() 
{
    // Initialize ROS
    this->node_handle = ros::NodeHandle();

    //Subscribers
    this->laser_sub = node_handle.subscribe("base_scan", 10, &AmazebotController::laserCallback, this);
	this->pose_sub = n.subscribe("/pose", 10, poseCallback);
	this->front_distance_sub = n.subscribe("/front_distance", 10, front_distance_callback);
	this->right_distance_sub = n.subscribe("/right_distance", 10, right_distance_callback);
	this->left_distance_sub = n.subscribe("/left_distance", 10, left_distance_callback);

	//Publishers
    this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	this->odom_pub = n.advertise<nav_msgs::Odometry>("/odom",10 );
    this->rgb_leds_pub = n.advertise<std_msgs::UInt8MultiArray>("/rgb_leds",60);
	this->reset_pose_pub = n.advertise<geometry_msgs::Pose2D>("/initial_pose",10);

    current_time = ros::Time::now();
  	last_time = ros::Time::now();
}

/**
 * @brief Run ROS
 * 
 */
void AmazebotController::run()
{
    // Send messages in a loop
    
    ros::Rate loop_rate(10); // Update rate of 10Hz
    while (ros::ok()) 
    {
        current_time = ros::Time::now();	
		double dt = (current_time - last_time).toSec();
        
        // Calculate the command to apply
        auto msg = calculateCommand();

        // Publish the new command
        this->cmd_vel_pub.publish(msg);

        ros::spinOnce();

        // And throttle the loop
        loop_rate.sleep();
    }
}