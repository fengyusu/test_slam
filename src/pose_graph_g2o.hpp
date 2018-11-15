//
// Created by fengyu on 18-11-15.
//

#ifndef TEST_SLAM_POSE_GRAPH_G_2_O_HPP
#define TEST_SLAM_POSE_GRAPH_G_2_O_HPP

#include "pose_graph_interface.hpp"
#include "g2o_slam2d/se2.h"






using namespace g2o;
using namespace g2o::tutorial;

class PoseGraphG2o: public PoseGraphInterface{
public:
    PoseGraphG2o();
    ~PoseGraphG2o() override;

    bool PoseGraphOptimization(int start_index, int opti_vertex_num,
                               int max_iteration_times = 100, double min_delta = 1.) ;

private:
    std::vector<SE2 *> vertex_se2_;
    std::vector<SE2 *> edge_se2_;


};


#endif //TEST_SLAM_POSE_GRAPH_G_2_O_HPP
