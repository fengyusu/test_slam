#ifndef TEST_SLAM_NODE_H
#define TEST_SLAM_NODE_H

#include <iostream>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "scan_matcher.hpp"
#include "pose_graph_custom.hpp"
#include "param_config.h"
#include "pose_graph_g2o.hpp"
#include "pose_graph_ceres.hpp"
#include "laser_data_process.hpp"

#include "slam/sensor_data_manager.h"
#include "slam/slam_processor.h"

class TestSlam
{
public:
    TestSlam(tf::TransformListener *tf);

    //进行时间同步
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;


    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& laserScanMsg);

    bool getTfPose(const std::string& target_frame_id,
                             const std::string& source_frame_id,
                             Eigen::Vector3d& pose, const ros::Time& t);


    void pubPathMsg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &path_pub_);


    void publishTf(ros::Time time_stamp);
    Eigen::Vector3d  calcuDeltaOdom(Eigen::Vector3d odom_pose);

    void MapPublishLoop(float map_pub_period);

public:
    Eigen::Vector3d scan_pos_cal;
    Eigen::Vector3d odom_pos_cal;

    ros::NodeHandle nh_;
    tf::TransformListener *tf_;
    tf::TransformBroadcaster* tfB_;

    ros::Publisher scan_path_pub_;
    nav_msgs::Path path_scan;

    ros::Publisher odom_path_pub_;
    nav_msgs::Path path_odom;

    ros::Publisher optimization_path_pub_;
    nav_msgs::Path path_optimization_;

    ros::Time current_time_;

    Eigen::Vector3d now_pos_,last_pos_;
    std::vector<Eigen::Vector3d> odom_increments_;

    std::thread* mpa_pub_thread_;
    std::mutex map_mutex_;
    ros::Publisher map_publisher_;
    ros::Publisher map_metadata_publisher_;
    nav_msgs::GetMap::Response map_;
    bool first_map_;


    Eigen::Vector3d best_pose_ = Eigen::Vector3d::Zero();

private:
    ParamConfig param_;

    std::shared_ptr<LaserDataProcess> laser_data_process_;
    ScanMatcher scan_matcher_;
//    PoseGraphCustom pose_graph_;
//    PoseGraphG2o pose_graph_;
    PoseGraphCeres pose_graph_;

    int vertex_counter_;
    Eigen::Matrix3d info_matrix;


    std::unique_ptr<slam::SlamProcessor> slam_processer_;
    std::shared_ptr<slam::SensorDataManager> sensor_data_manager_;

};

#endif
