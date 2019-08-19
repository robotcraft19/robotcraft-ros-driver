#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"


class RobotController
{
private:
    ros::NodeHandle n;

    // Publishers
    ros::Publisher cmd_vel_pub;
    ros::Publisher odom_pub;
    ros::Publisher left_ir_pub;
    ros::Publisher front_ir_pub;
    ros::Publisher right_ir_pub;

    // Subscribers
    ros::Subscriber pose_sub;
    ros::Subscriber ir_left_sub;
    ros::Subscriber ir_front_sub;
    ros::Subscriber ir_right_sub;

    // Variables
    float left_ir, front_ir, right_ir;
    float linear_vel, angular_vel;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        // TODO: Control code goes here
        msg.linear.x = linear_vel; // move forward (m/s -> unit of measure convention)
        msg.angular.z = angular_vel; // turn counterclockwise (rad/s -> unit of measure convention)

        return msg;
    }

    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
    {
        // Save pose, convert and publish to /odom topic

        // Broadcast tf transform
    }

    void leftIRCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        // Save left IR distance and republish
        left_ir = msg->data;

        // Publish left distance sensor
        auto ir_msg = sensor_msgs::Range();
        ir_msg.header.stamp = ros::Time::now();
        ir_msg.header.frame_id = "left_ir";
        ir_msg.radiation_type = 1;                    
        ir_msg.field_of_view = 0.034906585;
        ir_msg.min_range = 0.1;
        ir_msg.max_range = 0.8;
        ir_msg.range = left_ir;
        left_ir_pub.publish(ir_msg); 
    }
    void frontIRCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        // Save front IR distance and republish
        front_ir = msg->data;

        // Publish front distance sensor
        auto ir_msg = sensor_msgs::Range();
        ir_msg.header.stamp = ros::Time::now();
        ir_msg.header.frame_id = "front_ir";
        ir_msg.radiation_type = 1;                    
        ir_msg.field_of_view = 0.034906585;
        ir_msg.min_range = 0.1;
        ir_msg.max_range = 0.8;
        ir_msg.range = front_ir;
        front_ir_pub.publish(ir_msg); 
    }
    void rightIRCallback(const std_msgs::Float32::ConstPtr& msg)
    {
        // Save right IR distance and republish
        right_ir = msg->data;

        // Publish front distance sensor
        auto ir_msg = sensor_msgs::Range();
        ir_msg.header.stamp = ros::Time::now();
        ir_msg.header.frame_id = "right_ir";
        ir_msg.radiation_type = 1;                   
        ir_msg.field_of_view = 0.034906585;
        ir_msg.min_range = 0.1;
        ir_msg.max_range = 0.8;
        ir_msg.range = right_ir;
        right_ir_pub.publish(ir_msg); 
    }

public:
    RobotController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Setup publishers
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 10); // size of queue
        this->left_ir_pub = this->n.advertise<sensor_msgs::Range>("ir_left_sensor", 10);
        this->front_ir_pub = this->n.advertise<sensor_msgs::Range>("ir_front_sensor", 10);
        this->right_ir_pub = this->n.advertise<sensor_msgs::Range>("ir_right_sensor", 10);

        // Setup subscribers
        this->pose_sub = this->n.subscribe("pose", 10, &RobotController::poseCallback, this);
        this->ir_left_sub = this->n.subscribe("left_distance", 10, &RobotController::leftIRCallback, this);
        this->ir_front_sub = this->n.subscribe("front_distance", 10, &RobotController::frontIRCallback, this);
        this->ir_right_sub = this->n.subscribe("right_distance", 10, &RobotController::rightIRCallback, this);

        // Read parameters from server
        auto priv_nh = ros::NodeHandle("~"); // node handle declared in private space
        priv_nh.getParam("linear_vel", this->linear_vel);
        priv_nh.getParam("angular_vel", this->angular_vel);
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "talker");

    // Create our controller object and run it
    auto controller = RobotController();
    controller.run();

    // And make good on our promise
    return 0;
}