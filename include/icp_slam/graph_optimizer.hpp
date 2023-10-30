#ifndef GRAPH_OPTIMIZER_HPP_
#define GRAPH_OPTIMIZER_HPP_

/**
 * @brief pose grapher optimizer with g2o
 */

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/block_solver.h>

class GraphOptimizer {

    public:
        GraphOptimizer();

    private:
        void addPoseToGraph();
        void addEdge();

        std::shared_ptr<g2o::SparseOptimizer> optimizer_;
        std::shared_ptr<g2o::OptimizationAlgorithmProperty> solver_property_;

};

#endif // GRAPH_OPTIMIZER_HPP_
