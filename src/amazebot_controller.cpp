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

void AmazebotController::moveForward(float distance)
{
    auto msg = geometry_msgs::Twist();
    msg.linear.x = 0.3;
    float start_x = poseRobot.x;
    float start_y = poseRobot.y;
    while ((distance - calcDistance(poseRobot.x, start_x, poseRobot.y, start_y) > 0.1) && (ros::ok))
    {
        cmd_vel_pub.publish(msg);
        /*if (frontIR < 0.15) 
        {
            ROS_WARN("Warning : Obstacle at %f meters detected by front sensor", frontIR);
            if (frontIR < 0.1) 
            {
                this->stopRobot();
                this->loop_rate.sleep();
            }
        }*/
    }
    this->stopRobot();
}

void AmazebotController::moveBackwards(float distance)
{
    auto msg = geometry_msgs::Twist();
    msg.linear.x = -0.3;
    float start_x = poseRobot.x;
    float start_y = poseRobot.y;
    while (((distance - 0.5) - calcDistance(poseRobot.x, start_x, poseRobot.y, start_y) > 0.1) && (ros::ok))
    {
        cmd_vel_pub.publish(msg);
        this->loop_rate.sleep();
    }
    this->stopRobot();
}

void AmazebotController::turnLeft(int angle)
{
    auto msg = geometry_msgs::Twist();
    msg.angular.z = degToRad(angle / 4);
    int start_angle = radToDeg(poseRobot.theta);
    int end_angle = (start_angle + angle) % 360;
    if ( radToDeg(poseRobot.theta) < end_angle) 
    {
        while (radToDeg(poseRobot.theta) < end_angle && ros::ok)
        {
            cmd_vel_pub.publish(msg);
            this->loop_rate.sleep();
        }
    } 
    
    else if (radToDeg(poseRobot.theta) > end_angle)
    {
        while (radToDeg(poseRobot.theta) > 0 && ros::ok && radToDeg(poseRobot.theta) > end_angle)
        {
            cmd_vel_pub.publish(msg);
            this->loop_rate.sleep();
        }

        while (radToDeg(poseRobot.theta) < end_angle && ros::ok)
        {
            cmd_vel_pub.publish(msg);
            this->loop_rate.sleep();
        }
    }

    this->stopRobot();
}

void AmazebotController::turnRight(int angle)
{
    auto msg = geometry_msgs::Twist();
    msg.angular.z = -degToRad(angle / 4);
    float start_angle = poseRobot.theta;
    float end_angle = fmod((start_angle + angle), (2 * M_PI));
    if ( poseRobot.theta < end_angle) 
    {
        while (poseRobot.theta < end_angle && ros::ok)
        {
            cmd_vel_pub.publish(msg);
            this->loop_rate.sleep();
        }
    } 
    
    else if (poseRobot.theta > end_angle)
    {
        while (poseRobot.theta > 0 && ros::ok && poseRobot.theta > end_angle)
        {
            cmd_vel_pub.publish(msg);
            this->loop_rate.sleep();
        }

        while (poseRobot.theta < end_angle && ros::ok)
        {
            cmd_vel_pub.publish(msg);
            this->loop_rate.sleep();
        }
    }

    this->stopRobot();
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

    // Setting new pose
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
AmazebotController::AmazebotController() : loop_rate(10)
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
    this->laser_sub = node_handle.subscribe("base_scan", 10, &AmazebotController::laserCallback, this);
	this->pose_sub = node_handle.subscribe("/pose", 10, &AmazebotController::poseCallback, this);
	this->front_distance_sub = node_handle.subscribe("/front_distance", 10, &AmazebotController::frontDistanceCallback, this);
	this->right_distance_sub = node_handle.subscribe("/right_distance", 10, &AmazebotController::rightDistanceCallback, this);
	this->left_distance_sub = node_handle.subscribe("/left_distance", 10, &AmazebotController::leftDistanceCallback, this);

	//Publishers
    this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
	this->odom_pub = node_handle.advertise<nav_msgs::Odometry>("/odom", 10);
    this->rgb_leds_pub = node_handle.advertise<std_msgs::UInt8MultiArray>("/rgb_leds", 60);
	this->initial_pose_pub = node_handle.advertise<geometry_msgs::Pose2D>("/initial_pose", 10);
	this->ir_front_pub = node_handle.advertise<sensor_msgs::Range>("/ir_front_sensor",10);
	this->ir_left_pub  = node_handle.advertise<sensor_msgs::Range>("/ir_left_sensor",10);
    this->ir_right_pub = node_handle.advertise<sensor_msgs::Range>("/ir_right_sensor",10);

}

void AmazebotController::sensorHelper() 
{
    //publish front distance sensor
	ir_front_msg.header.stamp = ros::Time::now();
	ir_front_msg.header.frame_id = "front_ir";
	ir_front_msg.radiation_type = 1,                    
	ir_front_msg.field_of_view = 0.034906585;
	ir_front_msg.min_range = 0.1;
	ir_front_msg.max_range = 0.8;
	ir_front_msg.range = frontIR;
	ir_front_pub.publish(ir_front_msg);	
	
	//publish left distance sensor
	ir_left_msg.header.stamp = ros::Time::now();
	ir_left_msg.header.frame_id = "left_ir";
	ir_left_msg.radiation_type = 1,                    
	ir_left_msg.field_of_view = 0.034906585;
	ir_left_msg.min_range = 0.1;
	ir_left_msg.max_range = 0.8;
	ir_left_msg.range = leftIR;
	ir_left_pub.publish(ir_left_msg);
	
	//publish right distance sensor
	ir_right_msg.header.stamp = ros::Time::now();
	ir_right_msg.header.frame_id = "right_ir";
	ir_right_msg.radiation_type = 1,                    
	ir_right_msg.field_of_view = 0.034906585;
	ir_right_msg.min_range = 0.1;
	ir_right_msg.max_range = 0.8;
	ir_right_msg.range = rightIR;
	ir_right_pub.publish(ir_right_msg);				

	//publish rgb led data
	rgb_leds_msg.data.clear();
    rgb_leds_msg.data.push_back(Led1_R);
	rgb_leds_msg.data.push_back(Led1_G);
	rgb_leds_msg.data.push_back(Led1_B);
	rgb_leds_msg.data.push_back(Led2_R);
	rgb_leds_msg.data.push_back(Led2_G);
	rgb_leds_msg.data.push_back(Led2_B);
	rgb_leds_pub.publish(rgb_leds_msg);	
}

void AmazebotController::square_test()
{
    current_time = ros::Time::now();
  	last_time = ros::Time::now();
    // Send messages in a this->loop	
    //this->odometryHelper();
    //this->sensorHelper();

    // Calculate the command to apply
    this->turnLeft(90);
    this->moveForward(0.5);
    this->turnRight(90);
    this->moveForward(0.5);
    this->turnRight(90);
    this->moveForward(0.5);
    this->turnRight(90);
    this->moveForward(0.5);
    this->turnRight(90);

    last_time = current_time;
    ros::spinOnce();
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
        current_time = ros::Time::now();	
		
        this->odometryHelper();
        this->sensorHelper();

        // Calculate the command to apply
        auto msg = calculateCommand();

        // Publish the new command
        this->cmd_vel_pub.publish(msg);


        this->initialPose();
        last_time = current_time;
        ros::spinOnce();

        // And throttle the this->loop
        this->loop_rate.sleep();
    }
}
