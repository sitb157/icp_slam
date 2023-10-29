#include "icp_slam/graph_optimizer.hpp"

// Initialize
Graph_Optimizer::Graph_Optimizer() {
    graph_ = std::make_shared<g2o::SparseOptimizer>();
}
