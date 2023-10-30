#include "icp_slam/graph_optimizer.hpp"

// Initialize
GraphOptimizer::GraphOptimizer() {

    optimizer_ = std::make_shared<g2o::SparseOptimizer>();
    solver_property_ = std::make_shared<g2o::OptimizationAlgorithmProperty>();
    optimizer_->setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct("lm_var", *solver_property_));
    optimizer_->initializeOptimization();
    optimizer_->optimize(10);

}
