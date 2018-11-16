//
// Created by fengyu on 18-11-16.
//

#ifndef TEST_SLAM_LIDAR_CALIBRATE_HPP
#define TEST_SLAM_LIDAR_CALIBRATE_HPP

#include "param_config.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

class LaserDataProcess {
public:
    LaserDataProcess(ros::NodeHandle* nh, tf::TransformListener* tf, ParamConfig* param)
    {
        nh_ = nh;
        tf_ = tf;
        param_ = param;
        scan_pub_ = nh_->advertise<sensor_msgs::LaserScan>("/scan_calibrated", 10);
    }


    ~LaserDataProcess()
    {

    }

    void ScanDataCalibrate(sensor_msgs::LaserScan & scan_msg);



private:
    bool GetLaserPose(tf::Stamped<tf::Pose> &odom_pose, ros::Time dt);
    void StartCalibration(std::vector<double>& ranges,
                          std::vector<double>& angles,
                          ros::Time startTime,
                          ros::Time endTime);
    void BeamUpdate(tf::Stamped<tf::Pose> frame_base_pose,
            tf::Stamped<tf::Pose> frame_start_pose,
            tf::Stamped<tf::Pose> frame_end_pose,
            std::vector<double>& ranges,
            std::vector<double>& angles,
            int startIndex,
            int& beam_number);

    tf::TransformListener* tf_;
    ros::NodeHandle* nh_;
    ros::Publisher scan_pub_;

    ParamConfig* param_;
};


#endif //TEST_SLAM_LIDAR_CALIBRATE_HPP
