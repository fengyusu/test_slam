//
// Created by fengyu on 18-11-14.
//
#include <ros/ros.h>

#ifndef TEST_SLAM_PARAM_CONFIG_H
#define TEST_SLAM_PARAM_CONFIG_H

struct ParamConfig {
    void GetParam(ros::NodeHandle *nh) {
        nh->param<std::string>("odom_frame_id", odom_frame_id, "odom");
        nh->param<std::string>("laser_frame_id", laser_frame_id, "base_laser");
        nh->param<std::string>("base_frame_id", base_frame_id, "base_link");
        nh->param<std::string>("global_frame_id", global_frame_id, "odom");
        nh->param<std::string>("odom_topic_name", odom_topic_name, "odom");
        nh->param<std::string>("laser_topic_name", laser_topic_name, "scan");
        nh->param<std::string>("map_topic_name", pub_map_topic_name, "map");
        nh->param<bool>("publish_visualize", publish_visualize, true);
    }

    std::string odom_frame_id;
    std::string laser_frame_id;
    std::string base_frame_id;
    std::string global_frame_id;

    std::string odom_topic_name;
    std::string laser_topic_name;
    std::string pub_map_topic_name;




    bool publish_visualize;


};






#endif //TEST_SLAM_PARAM_CONFIG_H
