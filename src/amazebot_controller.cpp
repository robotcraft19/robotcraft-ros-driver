/**
 * @file robot_controller.cpp
 * @author Nicolas Filliol <nicolas.filliol@icloud.com>, Erwin Lejeune <erwin.lejeune15@gmail.com>, 
 *         Oleksandr Koreiba<alex@koreiba.com>, Jan Tiepelt, 
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
void AmazebotController::poseCallback(const geometry_msgs::Pose2D& Pose2D_msgs)
{
    poseRobot.x = Pose2D_msgs.x;
	poseRobot.y = Pose2D_msgs.y;
	poseRobot.theta = Pose2D_msgs.theta;
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
 * @brief 
 * 
 */
void AmazebotController::odometryHelper() 
{
    // Update the Pose to publish in the Odometry
    double dt = (current_time - last_time).toSec();
    double delta_x = (velocityRobot.x * cos(poseRobot.theta) - velocityRobot.y * sin(velocityRobot.theta)) * dt;
    double delta_y = (velocityRobot.x * sin(poseRobot.theta) - velocityRobot.y * cos(velocityRobot.theta)) * dt;
    double delta_theta = velocityRobot.theta * dt;

    poseRobot.x += delta_x;
    poseRobot.y += delta_y;
    poseRobot.theta += delta_theta;

    // Setting up the Transforn with the Odometry updates
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(poseRobot.theta);    
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
  	odom_trans.transform.translation.x = (poseRobot.x)/100;
    odom_trans.transform.translation.y = (poseRobot.y)/100;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // Sending the transform
    odom_broadcaster.sendTransform(odom_trans);

    // Seting new pose
	odom_msg.header.stamp = current_time;
	odom_msg.header.frame_id = "odom";
	odom_msg.pose.pose.position.x = poseRobot.x;
	odom_msg.pose.pose.position.y = poseRobot.y;
 	odom_msg.pose.pose.position.z = 0.0;
	odom_msg.pose.pose.orientation = odom_quat;

    // Setting velocity
	odom_msg.child_frame_id = "base_link";
    odom_msg.twist.twist.linear.x = velocityRobot.x;
    odom_msg.twist.twist.linear.y = velocityRobot.y;
    odom_msg.twist.twist.angular.z = velocityRobot.theta;

	// Publish the odometry message
	odom_pub.publish(odom_msg);  
}

/**
 * @brief 
 * 
 */
void AmazebotController::initialPose() 
{
    // Publish initial pose data
	initial_pose_msg.x = initialPoseRobot.x;
	initial_pose_msg.y = initialPoseRobot.y;
	initial_pose_msg.theta = initialPoseRobot.theta;		
	initial_pose_pub.publish(initial_pose_msg);
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
	this->pose_sub = node_handle.subscribe("/pose", 10, &AmazebotController::poseCallback, this);
	this->front_distance_sub = node_handle.subscribe("/front_distance", 10, &AmazebotController::frontDistanceCallback, this);
	this->right_distance_sub = node_handle.subscribe("/right_distance", 10, &AmazebotController::rightDistanceCallback, this);
	this->left_distance_sub = node_handle.subscribe("/left_distance", 10, &AmazebotController::leftDistanceCallback, this);

	//Publishers
    this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	this->odom_pub = node_handle.advertise<nav_msgs::Odometry>("/odom", 10);
    this->rgb_leds_pub = node_handle.advertise<std_msgs::UInt8MultiArray>("/rgb_leds", 60);
	this->initial_pose_pub = node_handle.advertise<geometry_msgs::Pose2D>("/initial_pose", 10);

    initialPoseRobot.x = initialPoseRobot.y = initialPoseRobot.theta = 0;
    velocityRobot.x = 0.1;
    velocityRobot.y = 0.0;
    velocityRobot.theta = 0.0;
}

/**
 * @brief Run ROS
 * 
 */
void AmazebotController::run()
{
    current_time = ros::Time::now();
  	last_time = ros::Time::now();
    // Send messages in a loop
    ros::Rate loop_rate(10); // Update rate of 10Hz
    while (ros::ok()) 
    {
        current_time = ros::Time::now();	
		
        this->odometryHelper();

        // Calculate the command to apply
        auto msg = calculateCommand();

        // Publish the new command
        this->cmd_vel_pub.publish(msg);


        this->initialPose();
        last_time = current_time;
        ros::spinOnce();

        // And throttle the loop
        loop_rate.sleep();
    }
}
