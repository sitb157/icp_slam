#ifndef GRAPH_OPTIMIZER_HPP_
#define GRAPH_OPTIMIZER_HPP_

/**
 * @brief pose grapher optimizer with g2o
 */

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

class GraphOptimizer {

    public:
        GraphOptimizer();

    private:
        void addPoseToGraph();
        void addEdge();

        std::shared_ptr<g2o::SparseOptimizer> optimizer_;
        g2o::OptimizationAlgorithmLevenberg* solver_;

};

#endif // GRAPH_OPTIMIZER_HPP_
