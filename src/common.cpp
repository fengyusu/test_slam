//
// Created by fengyu on 18-11-14.
//
#include "common.hpp"

Eigen::Matrix3d PoseToTrans(Eigen::Vector3d pose)
{
    Eigen::Matrix3d trans;
    trans << cos(pose(2)), -sin(pose(2)), pose(0),
            sin(pose(2)), cos(pose(2)), pose(1),
            0, 0, 1;

    return trans;
}

Eigen::Vector3d TransToPose(Eigen::Matrix3d trans)
{
    Eigen::Vector3d pose;
    pose(0) = trans(0,2);
    pose(1) = trans(1,2);
    pose(2) = atan2(trans(1,0), trans(0,0));

    return pose;
}
