//
// Created by fengyu on 18-11-14.
//
#include <csm/csm_all.h>
#include "sensor_msgs/LaserScan.h"
#include <eigen3/Eigen/Core>

#ifndef TEST_SLAM_SCANMATCHER_H
#define TEST_SLAM_SCANMATCHER_H

struct RangeData{


};

class ScanMatcher {
public:
    ScanMatcher();

    Eigen::Vector3d ResolveScanMatch(sensor_msgs::LaserScan *pScan,
                                                  Eigen::Vector3d init_trans);

private:

    void SetPIICPParams();
    void LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                        LDP& ldp);
    Eigen::Vector3d  PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                           Eigen::Vector3d tmprPose);

    //进行PI-ICP需要的变量
    LDP prev_ldp_;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;

};


#endif //TEST_SLAM_SCANMATCHER_H
