//
// Created by ray on 19-9-4.
//
#include "g2o_solver.h"
#include <g2o/stuff/misc.h>
#include <iostream>
#include <cmath>


using namespace std;
using namespace g2o;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;

typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;


G2OSolver::G2OSolver()
{
    // Initialize the SparseOptimizer
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();

    linearSolver->setBlockOrdering(false);

    auto blockSolver = g2o::make_unique<SlamBlockSolver>(std::move(linearSolver));

    optimizer_.setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver)));

    latestNodeID_ = 0;

    useRobustKernel_ = true;
}

G2OSolver::~G2OSolver()
{
    // destroy all the singletons
    g2o::Factory::destroy();

    g2o::OptimizationAlgorithmFactory::destroy();

    g2o::HyperGraphActionLibrary::destroy();
}

void G2OSolver::Clear()
{
    corrections_.clear();
}

const vector<MNode>& G2OSolver::GetCorrections() const
{
    return corrections_;
}

void G2OSolver::Compute(int end_id)
{
    corrections_.clear();

    // Fix the first node in the graph to hold the map in place
    g2o::OptimizableGraph::Vertex* first = optimizer_.vertex(0);
    g2o::OptimizableGraph::Vertex* end = optimizer_.vertex(end_id);
    if(!first)
    {
        ROS_ERROR("[g2o] No Node with ID 0 found!");
        return;
    }

    first->setFixed(true);
    if(!end)
    {
        ROS_ERROR("[g2o] No Node with ID 0 found!");
        return;
    }

    end->setFixed(true);
    // Do the graph optimization
    optimizer_.initializeOptimization();

    int iter = optimizer_.optimize(100);

    if (iter > 0)
    {
        ROS_INFO("[g2o] Optimization finished after %d iterations.", iter);
    }
    else
    {
        ROS_ERROR("[g2o] Optimization failed, result might be invalid!");
        return;
    }

    // Write the result so it can be used by the mapper
    g2o::SparseOptimizer::VertexContainer nodes = optimizer_.activeVertices();

    for (g2o::SparseOptimizer::VertexContainer::const_iterator n = nodes.begin(); n != nodes.end(); n++)
    {

        double estimate[3];

        if((*n)->getEstimateData(estimate))
        {
            //karto::Pose2 pose(estimate[0], estimate[1], estimate[2]);
            int id=(*n)->id();
            Eigen::Vector2d t(estimate[0],estimate[1]);
            double theta=estimate[2];
            MNode pose(id,t,theta);

            corrections_.emplace_back(pose);
        }
        else
        {
            ROS_ERROR("[g2o] Could not get estimated pose from Optimizer!");
        }
    }
}

void G2OSolver::AddNode(MNode* pose)
{

    //karto::Pose2 odom = pVertex->GetObject()->GetCorrectedPose();

    g2o::VertexSE2* poseVertex = new g2o::VertexSE2;

    poseVertex->setEstimate(g2o::SE2(pose->t[0], pose->t[1], pose->theta));

    poseVertex->setId(pose->id);

    //ROS_INFO("[loop] id %d add node. pose is (%f,%f,%f)",pose->id,pose->t[0],pose->t[1],pose->theta);

    optimizer_.addVertex(poseVertex);

    latestNodeID_ = pose->id;

    ROS_DEBUG("[g2o] Adding node %d.", pose->id);

}

void G2OSolver::AddConstraint(MEdge* edge){

    // Create a new edge
    g2o::EdgeSE2* odometry = new g2o::EdgeSE2;

    // Set source and target
    int sourceID = edge->from_id;

    int targetID = edge->to_id;

    odometry->vertices()[0] = optimizer_.vertex(sourceID);

    odometry->vertices()[1] = optimizer_.vertex(targetID);

    if(odometry->vertices()[0] == NULL)
    {

        ROS_ERROR("[g2o] Source vertex with id %d does not exist!", sourceID);

        delete odometry;

        return;
    }
    if(odometry->vertices()[0] == NULL)
    {

        ROS_ERROR("[g2o] Target vertex with id %d does not exist!", targetID);

        delete odometry;

        return;
    }

    // Set the measurement (odometry distance between vertices)
//    karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());
//
//    karto::Pose2 diff = pLinkInfo->GetPoseDifference();

    g2o::SE2 from_pose(edge->from_pose);
    g2o::SE2 to_pose(edge->to_pose);
    g2o::SE2 measurement(from_pose.inverse()*to_pose);
    //g2o::SE2 measurement(edge->t[0], edge->t[1], edge->theta);

    odometry->setMeasurement(measurement);

    //ROS_INFO("[loop] id %d and id %d add edge. pose is (%f,%f,%f)",sourceID,targetID,measurement.toVector()[0],measurement.toVector()[1],measurement.toVector()[2]);

    // Set the covariance of the measurement
//    karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
//
//    Eigen::Matrix<double,3,3> info;
//
//    info(0,0) = precisionMatrix(0,0);
//
//    info(0,1) = info(1,0) = precisionMatrix(0,1);
//
//    info(0,2) = info(2,0) = precisionMatrix(0,2);
//
//    info(1,1) = precisionMatrix(1,1);
//
//    info(1,2) = info(2,1) = precisionMatrix(1,2);
//
//    info(2,2) = precisionMatrix(2,2);
//
//    odometry->setInformation(info);


//    Eigen::Vector2d transNoise(0.1, 0.1);
//    double rotNoise = DEG2RAD(0.2);
//    if(targetID==0) {
//        transNoise[0] = 1.0;
//        transNoise[1] = 1.0;
//        rotNoise=1;
//    }
//    Eigen::Matrix3d covariance=Eigen::Matrix3d::Identity();
//    covariance.fill(0.);
//    covariance(0, 0) = transNoise[0]*transNoise[0];
//    covariance(1, 1) = transNoise[1]*transNoise[1];
//    covariance(2, 2) = rotNoise*rotNoise;

//    Eigen::Matrix3d information = covariance.inverse();
    Eigen::Matrix3d information = edge->covariance.inverse();

    //Eigen::Matrix<double,3,3> info=information;
    odometry->setInformation(information);

    if(useRobustKernel_)
    {

        g2o::RobustKernelDCS* rk = new g2o::RobustKernelDCS;

        odometry->setRobustKernel(rk);

    }

    // Add the constraint to the optimizer
    ROS_DEBUG("[g2o] Adding Edge from node %d to node %d.", sourceID, targetID);

    optimizer_.addEdge(odometry);
}