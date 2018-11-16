
#include "ros/ros.h"
#include "test_slam_node.hpp"




int main(int argc,char** argv)
{

    ros::init(argc, argv, "test_slam_node");
    ros::start();

    tf::TransformListener tf_buffer(ros::Duration(10));
    TestSlam slam(&tf_buffer);

    ros::spin();


    return 0;
}
