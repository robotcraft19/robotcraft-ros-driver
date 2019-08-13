/**
 * @file amazebot.cpp
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


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "amazebot_controller");

    // Create our controller object and run it
    auto controller = AmazebotController();
    controller.run();

    // And make good on our promise
    return 0;
}
