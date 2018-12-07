//
// Created by fengyu on 18-11-16.
//

#include "pose_graph_ceres.hpp"

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>


#include <ceres/ceres.h>



template <typename T>
Eigen::Matrix<T, 2, 2> RotationMatrix2D(T yaw_radians) {
    const T cos_yaw = ceres::cos(yaw_radians);
    const T sin_yaw = ceres::sin(yaw_radians);

    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;
    return rotation;
}

template <typename T>
inline T NormalizeAngle(const T& angle_radians) {
    // Use ceres::floor because it is specialized for double and Jet types.
    T two_pi(2.0 * M_PI);
    return angle_radians -
           two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}



struct PoseGraph2dErrorTerm {
public:
    PoseGraph2dErrorTerm(double x_ab, double y_ab, double yaw_ab_radians,
                         const Eigen::Matrix3d& sqrt_information)
            : p_ab_(x_ab, y_ab),
              yaw_ab_radians_(yaw_ab_radians),
              sqrt_information_(sqrt_information) {}

    template <typename T>
    bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a,
                    const T* const x_b, const T* const y_b, const T* const yaw_b,
                    T* residuals_ptr) const {
        const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
        const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);

        Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals_map(residuals_ptr);

        residuals_map.template head<2>() = RotationMatrix2D(static_cast<T>(yaw_ab_radians_)).transpose() * (RotationMatrix2D(*yaw_a).transpose() * (p_b - p_a) - p_ab_.cast<T>());
        residuals_map(2) = NormalizeAngle(
                (*yaw_b - *yaw_a) - static_cast<T>(yaw_ab_radians_));

        // Scale the residuals by the square root information matrix to account for
        // the measurement uncertainty.
        residuals_map = sqrt_information_.template cast<T>() * residuals_map;

        return true;
    }

//    static ceres::CostFunction* Create(double x_ab, double y_ab,
//                                       double yaw_ab_radians,
//                                       const Eigen::Matrix3d& sqrt_information) {
//        return (new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1,
//                1, 1>(new PoseGraph2dErrorTerm(
//                x_ab, y_ab, yaw_ab_radians, sqrt_information)));
//    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    // The position of B relative to A in the A frame.
    const Eigen::Vector2d p_ab_;
    // The orientation of frame B relative to frame A.
    const double yaw_ab_radians_;
    // The inverse square root of the measurement covariance matrix.
    const Eigen::Matrix3d sqrt_information_;
};



// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const std::vector<Constraint2d>& constraints,
                              std::map<int, Pose2d>* poses,
                              ceres::Problem* problem) {
    CHECK(poses != NULL);
    CHECK(problem != NULL);
    if (constraints.empty()) {
        LOG(INFO) << "No constraints, no problem to optimize.";
        return;
    }

    ceres::LossFunction* loss_function = nullptr;
//    ceres::LocalParameterization* angle_local_parameterization =
//            AngleLocalParameterization::Create();

    for (std::vector<Constraint2d>::const_iterator constraints_iter =
            constraints.begin();
         constraints_iter != constraints.end(); ++constraints_iter) {
        const Constraint2d& constraint = *constraints_iter;

        std::map<int, Pose2d>::iterator pose_begin_iter =
                poses->find(constraint.id_begin);
        CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
        std::map<int, Pose2d>::iterator pose_end_iter =
                poses->find(constraint.id_end);
        CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

        const Eigen::Matrix3d sqrt_information =
                constraint.information.llt().matrixL();
        // Ceres will take ownership of the pointer.

        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<PoseGraph2dErrorTerm, 3, 1, 1, 1, 1, 1, 1>
                (new PoseGraph2dErrorTerm(constraint.x, constraint.y, constraint.yaw_radians, sqrt_information));
        problem->AddResidualBlock(
                cost_function, loss_function,
                &pose_begin_iter->second.x, &pose_begin_iter->second.y, &pose_begin_iter->second.yaw_radians,
                &pose_end_iter->second.x, &pose_end_iter->second.y, &pose_end_iter->second.yaw_radians);

//        problem->SetParameterization(&pose_begin_iter->second.yaw_radians,
//                                     angle_local_parameterization);
//        problem->SetParameterization(&pose_end_iter->second.yaw_radians,
//                                     angle_local_parameterization);
    }

    // The pose graph optimization problem has three DOFs that are not fully
    // constrained. This is typically referred to as gauge freedom. You can apply
    // a rigid body transformation to all the nodes and the optimization problem
    // will still have the exact same cost. The Levenberg-Marquardt algorithm has
    // internal damping which mitigate this issue, but it is better to properly
    // constrain the gauge freedom. This can be done by setting one of the poses
    // as constant so the optimizer cannot change it.
    std::map<int, Pose2d>::iterator pose_start_iter =
            poses->begin();
    CHECK(pose_start_iter != poses->end()) << "There are no poses.";
    problem->SetParameterBlockConstant(&pose_start_iter->second.x);
    problem->SetParameterBlockConstant(&pose_start_iter->second.y);
    problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem* problem) {
    CHECK(problem != NULL);

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    std::cout << summary.FullReport() << '\n';

    return summary.IsSolutionUsable();
}



bool PoseGraphCeres::PoseGraphOptimization(int start_index, int opti_vertex_num,
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


    std::map<int, Pose2d> poses;
    std::vector<Constraint2d> constraints;

    for (int i = start_index_; i <= end_index_; ++i) {
        Pose2d pose2d = {vertexs_[i].pose[0],vertexs_[i].pose[1],vertexs_[i].pose[2]};
        poses[i] = pose2d;
    }

    for(const auto & edge : edges_) {
        if (edge.xi < start_index_ || edge.xi > end_index_ ||
            edge.xj < start_index_ || edge.xj > end_index_) {
            continue;
        }

        Constraint2d constraint2d = {edge.xi, edge.xj,
                                     edge.measurement[0], edge.measurement[1], edge.measurement[2],
                                     edge.info_matrix };
        constraints.push_back(constraint2d);
    }


    ceres::Problem problem;
    BuildOptimizationProblem(constraints, &poses, &problem);

    SolveOptimizationProblem(&problem);

    for (int i = start_index_; i <= end_index_; ++i) {
        if(poses.count(i)){
            Eigen::Vector3d tmp_pose;
            tmp_pose << poses[i].x, poses[i].y, poses[i].yaw_radians;
            vertexs_[i].pose = tmp_pose;
        }
    }



}

