//
// Created by fengyu on 18-11-15.
//

#include "pose_graph_g2o.hpp"

#include "g2o_slam2d/vertex_se2.h"
#include "g2o_slam2d/vertex_point_xy.h"
#include "g2o_slam2d/edge_se2.h"
#include "g2o_slam2d/edge_se2_pointxy.h"
#include "g2o_slam2d/types_tutorial_slam2d.h"

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using SlamBlockSolver = BlockSolver< BlockSolverTraits<-1, -1> >;
using SlamLinearSolver = LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> ;

SlamLinearSolver* linear_solver_ = nullptr;
SlamBlockSolver* block_solver_ = nullptr;
OptimizationAlgorithmGaussNewton* solver_ = nullptr;

PoseGraphG2o::PoseGraphG2o():
        PoseGraphInterface(){

    linear_solver_ = new SlamLinearSolver();
    linear_solver_->setBlockOrdering(false);
    block_solver_ = new SlamBlockSolver(linear_solver_);
    solver_ = new OptimizationAlgorithmGaussNewton(block_solver_);

}

PoseGraphG2o::~PoseGraphG2o(){
    delete linear_solver_;
    delete block_solver_;
    delete solver_;
}


bool PoseGraphG2o::PoseGraphOptimization(int start_index, int opti_vertex_num,
                           int max_iteration_times, double min_delta){
    if(opti_vertex_num <= 2 || vertexs_.count(start_index) == 0 ||
       vertexs_.count(start_index + opti_vertex_num - 1) == 0){
        return false;
    }

    start_index_ = start_index;
    end_index_ = start_index_ + opti_vertex_num - 1;
    opti_vertex_num_ = opti_vertex_num;

    std::cout << "vertex num : " << vertexs_.size() << "    "
              << "edges num : " << edges_.size() << std::endl;


    linear_solver_ = new SlamLinearSolver();
    linear_solver_->setBlockOrdering(false);
    block_solver_ = new SlamBlockSolver(linear_solver_);
    solver_ = new OptimizationAlgorithmGaussNewton(block_solver_);

    SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver_);
    optimizer.setVerbose( true );


    for (int i = start_index_; i <= end_index_; ++i) {
        SE2 *v_se2 = new SE2(vertexs_[i].pose[0],vertexs_[i].pose[1],vertexs_[i].pose[2]);
        vertex_se2_.push_back(v_se2);
        VertexSE2* robot_pose =  new VertexSE2;
        robot_pose->setId(i);
        robot_pose->setEstimate(*v_se2);
        optimizer.addVertex(robot_pose);
    }

    for(const auto & edge : edges_) {
        if (edge.xi < start_index_ || edge.xi > end_index_ ||
            edge.xj < start_index_ || edge.xj > end_index_) {
            continue;
        }

        EdgeSE2* constraint = new EdgeSE2;
        constraint->vertices()[0] = optimizer.vertex(edge.xi);
        constraint->vertices()[1] = optimizer.vertex(edge.xj);
        SE2 *e_se2 = new SE2(edge.measurement[0],edge.measurement[1],edge.measurement[2]);
        edge_se2_.push_back(e_se2);
        constraint->setMeasurement(*e_se2);
        constraint->setInformation(edge.info_matrix);
        optimizer.addEdge(constraint);
    }

    VertexSE2* first_pose = dynamic_cast<VertexSE2*>(optimizer.vertex(start_index_));
    first_pose->setFixed(true);

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    for (int i = start_index_; i <= end_index_; ++i) {
        SE2 opti_pose = (dynamic_cast<VertexSE2*>(optimizer.vertex(i)))->estimate();
        vertexs_[i].pose = opti_pose.toVector();
    }

    for(auto vertex_se2 : vertex_se2_){
        delete vertex_se2;
    }
    vertex_se2_.clear();

    for(auto vertex_se2 : edge_se2_){
        delete vertex_se2;
    }
    edge_se2_.clear();

    optimizer.clear();

    Factory::destroy();
    OptimizationAlgorithmFactory::destroy();
    HyperGraphActionLibrary::destroy();







}