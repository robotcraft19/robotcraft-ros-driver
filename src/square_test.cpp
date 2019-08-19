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

#include "square_controller.h"

int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "SquareController");

    auto controller = SquareController();
    controller.run();
    
    return 0;
}