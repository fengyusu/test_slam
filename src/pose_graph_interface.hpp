//
// Created by fengyu on 18-11-14.
//

#ifndef TEST_SLAM_POSE_GRAPH_HPP
#define TEST_SLAM_POSE_GRAPH_HPP

#include <vector>
#include <map>

#include <eigen3/Eigen/Core>

typedef struct vertex
{
    Eigen::Vector3d pose;
}Vertex;

typedef struct edge
{
    int xi,xj;
    Eigen::Vector3d measurement;
    Eigen::Matrix3d info_matrix;
}Edge;


class PoseGraphInterface {
public:
    PoseGraphInterface();
    virtual ~PoseGraphInterface() {};

    PoseGraphInterface(const PoseGraphInterface&) = delete;
    PoseGraphInterface& operator=(const PoseGraphInterface&) = delete;


    bool AddVertex(const Vertex &vertex, int vertex_index);
    std::map<int, Vertex> GetVertex(void);
    bool AddEdge(const Edge &edge);

    virtual bool PoseGraphOptimization(int start_index, int opti_vertex_num,
                               int max_iteration_times = 100, double min_delta = 1.) = 0;




    std::map<int, Vertex> vertexs_;
    std::vector<Edge> edges_;

    int start_index_, end_index_, opti_vertex_num_;


};


#endif //TEST_SLAM_POSE_GRAPH_HPP
