//
// Created by fengyu on 18-11-14.
//

#include "pose_graph_interface.hpp"

PoseGraphInterface::PoseGraphInterface():
    start_index_(0), end_index_(0), opti_vertex_num_(0){

}


std::map<int, Vertex> PoseGraphInterface::GetVertex(void){
    return this->vertexs_;
}

bool PoseGraphInterface::AddVertex(const Vertex &vertex, int vertex_index){

    vertexs_[vertex_index] = vertex;

    return true;
}

bool PoseGraphInterface::AddEdge(const Edge &edge){
    edges_.push_back(edge);

    return true;
}


