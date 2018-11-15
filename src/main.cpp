
#include "ros/ros.h"
#include "test_slam_node.hpp"




int main(int argc,char** argv)
{

    ros::init(argc, argv, "test_slam_node");
    ros::start();


    TestSlam slam;

    ros::spin();


    return 0;
}
