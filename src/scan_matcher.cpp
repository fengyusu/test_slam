//
// Created by fengyu on 18-11-14.
//

#include "scan_matcher.hpp"

ScanMatcher::ScanMatcher(){

    prev_ldp_ = NULL;
    SetPIICPParams();
}


//设置PI-ICP的参数
void ScanMatcher::SetPIICPParams()
{
    //设置激光的范围
    m_PIICPParams.min_reading = 0.1;
    m_PIICPParams.max_reading = 8;

    //设置位姿最大的变化范围
    m_PIICPParams.max_angular_correction_deg = 20.0;
    m_PIICPParams.max_linear_correction = 1;

    //设置迭代停止的条件
    m_PIICPParams.max_iterations = 50;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.0000001;

    //设置correspondence相关参数
    m_PIICPParams.max_correspondence_dist = 1;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PIICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PIICPParams.orientation_neighbourhood = 10;

    //设置使用PI-ICP
    m_PIICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;
}



void ScanMatcher::LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                              LDP& ldp)
{
    int nPts = pScan->intensities.size();
    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        double dist = pScan->ranges[i];
        if(dist > 0.1 && dist < 20)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = dist;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        ldp->theta[i] = pScan->angle_min+pScan->angle_increment*i;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}


//求两帧之间的icp位姿匹配
Eigen::Vector3d  ScanMatcher::PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                                    Eigen::Vector3d tmprPose)
{
    prev_ldp_->odometry[0] = 0.0;
    prev_ldp_->odometry[1] = 0.0;
    prev_ldp_->odometry[2] = 0.0;

    prev_ldp_->estimate[0] = 0.0;
    prev_ldp_->estimate[1] = 0.0;
    prev_ldp_->estimate[2] = 0.0;

    prev_ldp_->true_pose[0] = 0.0;
    prev_ldp_->true_pose[1] = 0.0;
    prev_ldp_->true_pose[2] = 0.0;

    //设置匹配的参数值
    m_PIICPParams.laser_ref = prev_ldp_;
    m_PIICPParams.laser_sens = currentLDPScan;

    m_PIICPParams.first_guess[0] = tmprPose(0);
    m_PIICPParams.first_guess[1] = tmprPose(1);
    m_PIICPParams.first_guess[2] = tmprPose(2);

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PIICPParams,&m_OutputResult);

    //nowPose在lastPose中的坐标
    Eigen::Vector3d  rPose;
    if(m_OutputResult.valid)
    {
        //得到两帧激光之间的相对位姿
        rPose(0)=(m_OutputResult.x[0]);
        rPose(1)=(m_OutputResult.x[1]);
        rPose(2)=(m_OutputResult.x[2]);

//        std::cout <<"Iter:"<<m_OutputResult.iterations<<std::endl;
//        std::cout <<"Corr:"<<m_OutputResult.nvalid<<std::endl;
//        std::cout <<"Erro:"<<m_OutputResult.error<<std::endl;

//        std::cout <<"PI ICP GOOD"<<std::endl;
    }
    else
    {
        std::cout <<"PI ICP Failed!!!!!!!"<<std::endl;
        rPose = tmprPose;
    }


    //ld_free(m_prevLDP);

    prev_ldp_ = currentLDPScan;

    return rPose;
}

Eigen::Vector3d ScanMatcher::ResolveScanMatch(sensor_msgs::LaserScan *pScan,
                                              Eigen::Vector3d init_trans){
    Eigen::Vector3d d_scan = Eigen::Vector3d::Zero();
    LDP currentLDP;
    if(prev_ldp_ != NULL)
    {
        LaserScanToLDP(pScan,currentLDP);
        d_scan = PIICPBetweenTwoFrames(currentLDP, init_trans);
    }
    else
    {
        LaserScanToLDP(pScan,prev_ldp_);
    }

    return d_scan;
}
