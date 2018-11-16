//
// Created by fengyu on 18-11-16.
//

#ifndef TEST_SLAM_POSE_GRAPH_CERES_HPP
#define TEST_SLAM_POSE_GRAPH_CERES_HPP


#include "pose_graph_interface.hpp"


// The state for each vertex in the pose graph.
struct Pose2d {
    double x;
    double y;
    double yaw_radians;

    // The name of the data type in the g2o file format.
    static std::string name() {
        return "VERTEX_SE2";
    }
};


// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Constraint2d {
    int id_begin;
    int id_end;

    double x;
    double y;
    double yaw_radians;

    // The inverse of the covariance matrix for the measurement. The order of the
    // entries are x, y, and yaw.
    Eigen::Matrix3d information;

    // The name of the data type in the g2o file format.
    static std::string name() {
        return "EDGE_SE2";
    }
};



class PoseGraphCeres : public PoseGraphInterface{
public:

    PoseGraphCeres() : PoseGraphInterface(){};
    ~PoseGraphCeres() override {};

    bool PoseGraphOptimization(int start_index, int opti_vertex_num,
                               int max_iteration_times = 100, double min_delta = 1.) ;

private:
    /*
    void BuildOptimizationProblem(const std::vector<Constraint2d>& constraints,
                                                  std::map<int, Pose2d>* poses,
                                                  ceres::Problem* problem);
    bool SolveOptimizationProblem(ceres::Problem* problem);
*/

};


#endif //TEST_SLAM_POSE_GRAPH_CERES_HPP
