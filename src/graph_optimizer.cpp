#include "icp_slam/graph_optimizer.hpp"

// Initialize
GraphOptimizer::GraphOptimizer() {

    optimizer_ = std::make_shared<g2o::SparseOptimizer>();
    optimizer_->setVerbose(true);
    linear_solver_ = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    solver_ = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver_)));
    optimizer_->setAlgorithm(solver_);
}

void GraphOptimizer::addVertex(const g2o::SE2& pose, int id) {
    g2o::VertexSE2* vertex = new g2o::VertexSE2();
    vertex->setId(id);
    if (id == 0) {
        vertex->setFixed(true);
    }
    vertex->setEstimate(pose);
    optimizer_->addVertex(vertex);
}

void GraphOptimizer::addEdge() {
    //g2o::VertexSE2* vertex1 = ;
    //g2o::VertexSE2* vertex2 = ;
    //g2o::EdgeSE2* edge = new g2o::EdgeSE2();

    //edge->setVertex(0, vertex1);
    //edge->setVertex(1, vertex2);

    //edge->setMeasurement();
    //edge->setInformation();

    //optimizer_->addEdge(edge);
}

void GraphOptimizer::optimize() {
    optimizer_->initializeOptimization();
    optimizer_->optimize(100);
}
