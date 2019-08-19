/**
 * @file square_test.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-08-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "amazebot_controller.h"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "square_test");

    // Create our controller object and run it
    auto square = AmazebotController();
    square.square_test();

    // And make good on our promise
    return 0;
}