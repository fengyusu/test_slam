//
// Created by fengyu on 18-11-15.
//

#include "pose_graph_custom.hpp"
#include "common.hpp"

#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/LU>
#include <iostream>

PoseGraphCustom::PoseGraphCustom():
        PoseGraphInterface(){

}


double PoseGraphCustom::ComputeError(void) {
    double sum_error = 0;
    for(const auto edge : edges_) {
        if(edge.xi < start_index_ || edge.xi > end_index_ ||
           edge.xj < start_index_ || edge.xj > end_index_){
            continue;
        }

        Eigen::Vector3d xi = vertexs_[edge.xi].pose;
        Eigen::Vector3d xj = vertexs_[edge.xj].pose;
        Eigen::Vector3d z = edge.measurement;
        Eigen::Matrix3d info_matrix = edge.info_matrix;

        Eigen::Matrix3d Xi = PoseToTrans(xi);
        Eigen::Matrix3d Xj = PoseToTrans(xj);
        Eigen::Matrix3d Z  = PoseToTrans(z);

        Eigen::Matrix3d Ei = Z.inverse() *  Xi.inverse() * Xj;

        Eigen::Vector3d ei = TransToPose(Ei);


        sum_error += ei.transpose() * info_matrix * ei;
    }

    return sum_error;
}



void PoseGraphCustom::CalcJacobianAndError(Eigen::Vector3d xi,Eigen::Vector3d xj,Eigen::Vector3d z,
                                     Eigen::Vector3d& ei,Eigen::Matrix3d& Ai,Eigen::Matrix3d& Bi) {

    Eigen::Matrix3d Xi = PoseToTrans(xi);
    Eigen::Matrix3d Xj = PoseToTrans(xj);
    Eigen::Matrix3d Z  = PoseToTrans(z);

    Eigen::Matrix3d Ei = Z.inverse() *  Xi.inverse() * Xj;
    ei = TransToPose(Ei);

    double theta_i = xi(2);
    double theta_j = xj(2);
    double theta_ij = z(2);

    Eigen::Matrix2d Ri;
    Eigen::Matrix2d Rij;

    Eigen::Vector2d ti = xi.block(0,0,2,1);
    Eigen::Vector2d tj = xj.block(0,0,2,1);

    Ri << cos(theta_i), -sin(theta_i),
            sin(theta_i), cos(theta_i);

    Rij << cos(theta_ij), -sin(theta_ij),
            sin(theta_ij), cos(theta_ij);

    Ai << 0, 0, 0,
            0, 0, 0,
            0, 0, -1;
    Ai.block(0, 0, 2, 2) = -Rij.transpose() * Ri.transpose();
    Eigen::Matrix2d RiT_the;
    RiT_the << -sin(theta_i), cos(theta_i),
            -cos(theta_i),-sin(theta_i);
    Ai.block(0, 2, 2, 1) = Rij.transpose() * RiT_the * (tj - ti);

    Bi << 0, 0, 0,
            0, 0, 0,
            0, 0, 1;
    Bi.block(0, 0, 2, 2) = Rij.transpose() * Ri.transpose();


}

Eigen::VectorXd  PoseGraphCustom::LinearizeAndSolve(void) {
    Eigen::MatrixXd H(opti_vertex_num_ * 3, opti_vertex_num_ * 3);
    Eigen::VectorXd b(opti_vertex_num_ * 3);

    H.setZero();
    b.setZero();

    Eigen::Matrix3d I;
    I.setIdentity();
    H.block(0,0,3,3) += I;

    for(const auto edge : edges_){
        if(edge.xi < start_index_ || edge.xi > end_index_ ||
           edge.xj < start_index_ || edge.xj > end_index_){
            continue;
        }

        Eigen::Vector3d xi = vertexs_[edge.xi].pose;
        Eigen::Vector3d xj = vertexs_[edge.xj].pose;
        Eigen::Vector3d z = edge.measurement;
        Eigen::Matrix3d info_matrix = edge.info_matrix;

        //计算误差和对应的Jacobian
        Eigen::Vector3d eij;
        Eigen::Matrix3d Aij;
        Eigen::Matrix3d Bij;
        CalcJacobianAndError(xi,xj,z,eij,Aij,Bij);


        int block_xi = 3 * (edge.xi - start_index_);
        int block_xj = 3 * (edge.xj - start_index_);
        H.block(block_xi, block_xi, 3, 3) += Aij.transpose() * info_matrix * Aij;
        H.block(block_xj, block_xi, 3, 3) += Bij.transpose() * info_matrix * Aij;
        H.block(block_xi, block_xj, 3, 3) += Aij.transpose() * info_matrix * Bij;
        H.block(block_xj, block_xj, 3, 3) += Bij.transpose() * info_matrix * Bij;

        b.block(block_xi, 0, 3, 1) += Aij.transpose() * info_matrix * eij;
        b.block(block_xj, 0, 3, 1) += Bij.transpose() * info_matrix * eij;

    }

    Eigen::VectorXd dx;
    dx = H.ldlt().solve(-b);


    return dx;
}



bool PoseGraphCustom::PoseGraphOptimization(int start_index, int opti_vertex_num,
                                      int max_iteration_times, double min_delta) {

    if(opti_vertex_num <= 2 || vertexs_.count(start_index) == 0 ||
       vertexs_.count(start_index + opti_vertex_num - 1) == 0){
        return false;
    }

    start_index_ = start_index;
    end_index_ = start_index_ + opti_vertex_num - 1;
    opti_vertex_num_ = opti_vertex_num;

    std::cout << "vertex num : " << vertexs_.size() << "    "
              << "edges num : " << edges_.size() << std::endl;

    double init_error = ComputeError();
    std::cout <<"initError:"<<init_error<<std::endl;

    for(int i = 0; i < max_iteration_times;i++) {
        std::cout << "Iterations:" << i;
        Eigen::VectorXd dx = LinearizeAndSolve();
        std::cout <<"    error:"<< ComputeError() << std::endl;

        for(int v_index = start_index_; v_index <= end_index_; v_index++) {
            vertexs_[v_index].pose += dx.block(3 * (v_index - start_index_), 0, 3, 1);
        }

        double max_error = -1;
        for(int k = 0; k < 3 * opti_vertex_num; k++) {
            if(max_error < std::fabs(dx(k))) {
                max_error = std::fabs(dx(k));
            }
        }

        if(max_error < min_delta){
            break;
        }

    }


    double final_error  = ComputeError();

    std::cout <<"FinalError:"<<final_error<<std::endl;


    return true;
}