//
// Created by fengyu on 18-11-15.
//

#ifndef TEST_SLAM_POSE_GRAPH_CUSTOM_HPP
#define TEST_SLAM_POSE_GRAPH_CUSTOM_HPP

#include "pose_graph_interface.hpp"

class PoseGraphCustom : public PoseGraphInterface{
public:
    PoseGraphCustom();
    ~PoseGraphCustom() override {};

    bool PoseGraphOptimization(int start_index, int opti_vertex_num,
                               int max_iteration_times = 100, double min_delta = 1.) ;


private:

    Eigen::VectorXd  LinearizeAndSolve(void);
    void CalcJacobianAndError(Eigen::Vector3d xi,Eigen::Vector3d xj,Eigen::Vector3d z,
                              Eigen::Vector3d& ei,Eigen::Matrix3d& Ai,Eigen::Matrix3d& Bi);
    double ComputeError(void);


};


#endif //TEST_SLAM_POSE_GRAPH_CUSTOM_HPP
