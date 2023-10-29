#ifndef GRAPH_OPTIMIZER_HPP_
#define GRAPH_OPTIMIZER_HPP_

/**
 * @brief pose grapher optimizer with g2o
 */

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>

class Graph_Optimizer {

    public:
        Graph_Optimizer();

    private:
        void addPoseToGraph();
        void addEdge();

        std::shared_ptr<g2o::SparseOptimizer> graph_;

};

#endif // GRAPH_OPTIMIZER_HPP_
