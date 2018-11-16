//
// Created by fengyu on 18-11-16.
//

#include "laser_data_process.hpp"




#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iostream>

void LaserDataProcess::ScanDataCalibrate(sensor_msgs::LaserScan & scan_msg)
{

    ros::Time startTime, endTime;
    startTime = scan_msg.header.stamp;

    sensor_msgs::LaserScan scan_msg_calibrated(scan_msg);

    int beamNum = scan_msg.ranges.size();
    endTime = startTime + ros::Duration(scan_msg.time_increment * beamNum);


    std::vector<double> angles,ranges;
    for(int i = beamNum - 1; i > 0;i--)
    {
        double lidar_dist = scan_msg.ranges[i];
        double lidar_angle = scan_msg.angle_min + scan_msg.angle_increment * i;

        if(lidar_dist < 0.05 || std::isnan(lidar_dist) || std::isinf(lidar_dist))
            lidar_dist = 0.0;

        ranges.push_back(lidar_dist);
        angles.push_back(lidar_angle);
    }

//    std::cout << "StartCalibration" << std::endl;
//    std::cout << "startTime  " << startTime << "  endTime  " << endTime << std::endl;
    StartCalibration(ranges, angles, startTime, endTime);
//    std::cout << "StartCalibration end" << std::endl;


    for(int i = 0; i < ranges.size(); i++)
    {
        scan_msg_calibrated.ranges[i] = 0.0f;
    }
    for(int i = 0; i < ranges.size(); i++)
    {
        int j = (angles[i] - scan_msg_calibrated.angle_min) / scan_msg_calibrated.angle_increment;
        if(j < 0)continue;
        scan_msg_calibrated.ranges[j] = ranges[i];
    }


    scan_pub_.publish(scan_msg_calibrated);
    scan_msg = scan_msg_calibrated;

}


void LaserDataProcess::StartCalibration(std::vector<double>& ranges,
                                             std::vector<double>& angles,
                                             ros::Time startTime,
                                             ros::Time endTime)
{

    int beamNumber = ranges.size();
    if(beamNumber != angles.size())
    {
        ROS_WARN("Error:ranges not match to the angles");
        return ;
    }

    int interpolation_time_duration = 5 * 1000;

    tf::Stamped<tf::Pose> frame_start_pose;
    tf::Stamped<tf::Pose> frame_mid_pose;
    tf::Stamped<tf::Pose> frame_base_pose;
    tf::Stamped<tf::Pose> frame_end_pose;


    double start_time = startTime.toSec() * 1000 * 1000;
    double end_time = endTime.toSec() * 1000 * 1000;
    double time_inc = (end_time - start_time) / beamNumber;


    int start_index = 0;



    if(!GetLaserPose(frame_start_pose, ros::Time(start_time /1000000.0)))
    {
        ROS_WARN("Not Start Pose,Can not Calib");
        return ;
    }

    if(!GetLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0)))
    {
        ROS_WARN("Not End Pose, Can not Calib");
        return ;
    }

    int cnt = 0;

    frame_base_pose = frame_start_pose;
    for(int i = 0; i < beamNumber; i++)
    {

        double mid_time = start_time + time_inc * (i - start_index);
        if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
        {
            cnt++;


            if(!GetLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0)))
            {
                ROS_WARN("Mid %d Pose Error",cnt);
                return ;
            }


            int interp_count = i - start_index + 1;

            BeamUpdate(frame_base_pose,
                                    frame_start_pose,
                                    frame_mid_pose,
                                    ranges,
                                    angles,
                                    start_index,
                                    interp_count);

            start_time = mid_time;
            start_index = i;
            frame_start_pose = frame_mid_pose;
        }
    }
}




void LaserDataProcess::BeamUpdate(
        tf::Stamped<tf::Pose> frame_base_pose,
        tf::Stamped<tf::Pose> frame_start_pose,
        tf::Stamped<tf::Pose> frame_end_pose,
        std::vector<double>& ranges,
        std::vector<double>& angles,
        int startIndex,
        int& beam_number)
{
    double frame_mid_yaw[beam_number],frame_mid_x[beam_number],frame_mid_y[beam_number];

    frame_mid_yaw[0] = tf::getYaw(frame_start_pose.getRotation());
    frame_mid_yaw[beam_number-1] = tf::getYaw(frame_end_pose.getRotation());
    double det_yaw = (frame_mid_yaw[beam_number-1] - frame_mid_yaw[0]) / (beam_number - 1);

    frame_mid_x[0] = frame_start_pose.getOrigin().getX();
    frame_mid_x[beam_number-1] = frame_end_pose.getOrigin().getX();
    double det_x = (frame_mid_x[beam_number-1] - frame_mid_x[0]) / (beam_number - 1);

    frame_mid_y[0] = frame_start_pose.getOrigin().getY();
    frame_mid_y[beam_number-1] = frame_end_pose.getOrigin().getY();
    double det_y = (frame_mid_y[beam_number-1] - frame_mid_y[0]) / (beam_number - 1);


    for(int i = 0; i < beam_number; i++)
    {
        frame_mid_yaw[i] = frame_mid_yaw[0] + det_yaw * (i);
        frame_mid_x[i] = frame_mid_x[0] + det_x * (i);
        frame_mid_y[i] = frame_mid_y[0] + det_y * (i);
    }

    double frame_base_yaw = -tf::getYaw(frame_base_pose.getRotation());
    double frame_base_x = frame_base_pose.getOrigin().getX();
    double frame_base_y = frame_base_pose.getOrigin().getY();

    for(int i = startIndex; i < startIndex + beam_number; i++)
    {
        double x = ranges[i] * cos(angles[i]);
        double y = ranges[i] * sin(angles[i]);


        double x_odom = x * cos(frame_mid_yaw[i - startIndex]) - y * sin(frame_mid_yaw[i - startIndex]) + frame_mid_x[i - startIndex];
        double y_odom = x * sin(frame_mid_yaw[i - startIndex]) + y * cos(frame_mid_yaw[i - startIndex]) + frame_mid_y[i - startIndex];

        double tmp_x = x_odom - frame_base_x;
        double tmp_y = y_odom - frame_base_y;

        frame_mid_x[i - startIndex] = tmp_x * cos(frame_base_yaw) - tmp_y * sin(frame_base_yaw);
        frame_mid_y[i - startIndex] = tmp_x * sin(frame_base_yaw) + tmp_y * cos(frame_base_yaw);

        ranges[i] = sqrt(frame_mid_x[i - startIndex] * frame_mid_x[i - startIndex] + frame_mid_y[i - startIndex] * frame_mid_y[i - startIndex]);
        angles[i] = atan2(frame_mid_y[i - startIndex], frame_mid_x[i - startIndex]);
    }

}

bool LaserDataProcess::GetLaserPose(tf::Stamped<tf::Pose> &odom_pose, ros::Time dt)
{
    odom_pose.setIdentity();

    tf::Stamped < tf::Pose > robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = param_->laser_frame_id;
    robot_pose.stamp_ = dt;


    try
    {
        if(!tf_->waitForTransform(param_->odom_frame_id, param_->laser_frame_id, dt, ros::Duration(0.5)))             // 0.15s 的时间可以修改
        {
            std::cout << "Can not Wait Transform" << std::endl;
            return false;
        }
        tf_->transformPose(param_->odom_frame_id, robot_pose, odom_pose);
    }
    catch (tf::LookupException& ex)
    {
        std::cout << "No Transform available" << std::endl;
        return false;
    }
    catch (tf::ConnectivityException& ex)
    {
        std::cout << "Connectivity Error" << std::endl;
        return false;
    }
    catch (tf::ExtrapolationException& ex)
    {
        std::cout << "Extrapolation Error" << std::endl;
        return false;
    }

    return true;
}
