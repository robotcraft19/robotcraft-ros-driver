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
        msg.angular.z = 1.25; // maximum value
        msg.linear.x = -0.04;
    } 
    else if (robot_lost == true)
    {
        // Robot is lost, go straight to find wall
        msg.linear.x = 0.08;
    } 
    
    else 
    {
        // Robot keeps using normal PID controller
        float gain = calculateGain(rightIR);
        msg.linear.x = 0.08;
        msg.angular.z = gain;
    }
    return msg;
}

/**
 * @brief 
 * 
 * @param front_distance_msgs 
 */
void AmazebotController::frontDistanceCallback(const sensor_msgs::Range& front_distance_msgs)
{
	frontIR = front_distance_msgs.range;
}

/**
 * @brief 
 * 
 * @param left_distance_msgs 
 */
void AmazebotController::leftDistanceCallback(const sensor_msgs::Range& left_distance_msgs)
{
	leftIR = left_distance_msgs.range;
}

/**
 * @brief 
 * 
 * @param right_distance_msgs 
 */
void AmazebotController::rightDistanceCallback(const sensor_msgs::Range& right_distance_msgs)
{
	rightIR = right_distance_msgs.range;
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

    ROS_INFO("Gain: %f, P: %f, I: %f, D: %f", gain, error, new_int_err, new_der_err);

    //if(gain > 0.5) gain = 0.3;
    if(gain < -0.4) gain = -0.4;

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

            if (lost_counter >= 50) robot_lost = true;
    } 
    else if((frontIR < THRESHOLD_DISTANCE && frontIR != -1) || (rightIR < THRESHOLD_DISTANCE && rightIR != -1)) 
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
	this->odom_sub = this->node_handle.subscribe("/odom", 10, &AmazebotController::odomCallback, this);
	this->front_distance_sub = this->node_handle.subscribe("/ir_front_sensor", 10, &AmazebotController::frontDistanceCallback, this);
	this->right_distance_sub = this->node_handle.subscribe("/ir_right_sensor", 10, &AmazebotController::rightDistanceCallback, this);
	this->left_distance_sub = this->node_handle.subscribe("/ir_left_sensor", 10, &AmazebotController::leftDistanceCallback, this);

	//Publishers
    this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    this->rgb_leds_pub = this->node_handle.advertise<std_msgs::UInt8MultiArray>("/rgb_leds", 10);
	this->set_pose_pub = this->node_handle.advertise<geometry_msgs::Pose2D>("/set_pose", 10);

    //Load Params
    this->node_handle.getParam("KD", this->KD);
    this->node_handle.getParam("KI", this->KI);
    this->node_handle.getParam("KP", this->KP);  
    ROS_WARN("KP: %f, KI: %f, KD: %f", this->KP, this->KI, this->KD);  

}

/**
 * @brief Run ROS
 * 
 */
void AmazebotController::run()
{
    ros::Rate loop_rate(10);

    // Send messages in a loop
    while (ros::ok()) 
    {
        // Calculate the command to apply
        auto msg = calculateCommand();

        // Publish the new command
        this->cmd_vel_pub.publish(msg);

        // Receive messages
        ros::spinOnce();

        // And throttle the loop
        loop_rate.sleep();
    }
}
