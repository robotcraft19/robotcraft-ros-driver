#include "amazebot_controller.h"

/**
 * @brief Run the robot controller node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
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