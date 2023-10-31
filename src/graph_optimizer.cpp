#include "icp_slam/graph_optimizer.hpp"

// Initialize
GraphOptimizer::GraphOptimizer() {

    optimizer_ = std::make_shared<g2o::SparseOptimizer>();
    optimizer_->setVerbose(true);
    linear_solver_ = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    solver_ = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver_)));
    optimizer_->setAlgorithm(solver_);
    optimizer_->initializeOptimization();
    optimizer_->optimize(10);

}
