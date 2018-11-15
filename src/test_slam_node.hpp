#ifndef TEST_SLAM_NODE_H
#define TEST_SLAM_NODE_H

#include <iostream>
#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "scan_matcher.hpp"
#include "pose_graph_custom.hpp"
#include "param_config.h"
#include "pose_graph_g2o.hpp"

class TestSlam
{
public:
    TestSlam();



    Eigen::Vector3d scan_pos_cal;
    Eigen::Vector3d odom_pos_cal;

    ros::NodeHandle nh_;
    tf::TransformListener tf_;

    ros::Publisher scan_path_pub_;
    nav_msgs::Path path_scan;

    ros::Publisher optimization_path_pub_;
    nav_msgs::Path path_optimization_;

    ros::Time current_time_;

    Eigen::Vector3d now_pos_,last_pos_;
    std::vector<Eigen::Vector3d> odom_increments_;

    //进行时间同步
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;


    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& laserScanMsg);

    bool getTfPose(const std::string& target_frame_id,
                             const std::string& source_frame_id,
                             Eigen::Vector3d& pose, const ros::Time& t);


    void pubPathMsg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &path_pub_);



    Eigen::Vector3d  calcuDeltaOdom(Eigen::Vector3d odom_pose);

private:
    ParamConfig param_;

    ScanMatcher scan_matcher_;
//    PoseGraphCustom pose_graph_;
    PoseGraphG2o pose_graph_;

    int vertex_counter_;
    Eigen::Matrix3d info_matrix;

};

#endif
