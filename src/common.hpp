//
// Created by fengyu on 18-11-14.
//

#ifndef TEST_SLAM_COMMON_HPP
#define TEST_SLAM_COMMON_HPP

#include <vector>
#include <eigen3/Eigen/Core>

Eigen::Matrix3d PoseToTrans(Eigen::Vector3d pose);

Eigen::Vector3d TransToPose(Eigen::Matrix3d trans);

#endif //TEST_SLAM_COMMON_HPP
