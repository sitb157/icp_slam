#ifndef GRAPH_OPTIMIZER_HPP_
#define GRAPH_OPTIMIZER_HPP_

/**
 * @brief pose grapher optimizer with g2o
 */

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

class GraphOptimizer {

    public:
        GraphOptimizer();

        void addVertex(const g2o::SE2&, int id);
        void addEdge();
        void optimize();

    private:

        std::shared_ptr<g2o::SparseOptimizer> optimizer_;
        std::unique_ptr<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>> linear_solver_;
        g2o::OptimizationAlgorithmLevenberg* solver_;

};

#endif // GRAPH_OPTIMIZER_HPP_
