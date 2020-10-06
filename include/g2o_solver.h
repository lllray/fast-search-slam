//
// Created by ray on 19-9-4.
//

#ifndef ICP_BNB_G2O_SOLVER_H
#define ICP_BNB_G2O_SOLVER_H

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/core/sparse_optimizer.h"

#include "g2o/stuff/macros.h"

#include <iostream>

#include <icp_type.h>
#include <vector>
#include <eigen3/Eigen/Core>

//#include "vertex_se2.h"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
//#include "g2o_types_slam2d_api.h"
G2O_USE_TYPE_GROUP(slam2d);
//G2O_USE_TYPE_GROUP(slam3d);

class G2OSolver{
 public:
    G2OSolver();
    ~G2OSolver();
    /**
     * @brief Clear the vector of corrections
     * @details Empty out previously computed corrections
     */
    void Clear();

    /**
     * @brief Solve the SLAM back-end
     * @details Calls G2O to solve the SLAM back-end
     */
    void Compute(int end_id);
    /**
     * @brief Get the vector of corrections
     * @details Get the vector of corrections
     * @return Vector with corrected poses
     */
    virtual const std::vector<MNode>& GetCorrections() const;

    /**
     * @brief Add a node to pose-graph
     * @details Add a node which is a robot pose to the pose-graph
     *
     * @param pVertex the node to be added in
     */

    void AddNode(MNode* pose);

    /**
     * @brief Add an edge constraint to pose-graph
     * @details Adds a relative pose measurement constraint between two poses in the graph
     *
     * @param pEdge [description]
     */
    void AddConstraint(MEdge* edge);

    /**
 * @brief Use robust kernel in back-end
 * @details Uses Dynamic Covariance scaling kernel in back-end
 *
 * @param flag variable, if true robust kernel will be used
 */
    void useRobustKernel(bool flag)
    {
        useRobustKernel_ = flag;
    }
private:
    g2o::SparseOptimizer optimizer_;

    std::vector<MNode> corrections_;

    int latestNodeID_; // ID of the latest added node, this is used internally in AddHeadingConstraint

    bool useRobustKernel_;
};

#endif //ICP_BNB_G2O_SOLVER_H
