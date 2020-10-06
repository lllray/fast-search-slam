//
// Created by ray on 19-10-24.
//

#ifndef ICP_BNB_ICP_TYPE_H
#define ICP_BNB_ICP_TYPE_H


#include <opencv2/opencv.hpp>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include "bits/stdc++.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

struct RobotOdom{
    double time;
    Eigen::Vector3d pose;
    bool use;

    RobotOdom(){time=0;use=false;}
};

struct MNode{
    int id;
    Eigen::Vector2d t;
    double theta;

    MNode(int i,Eigen::Vector2d tt,double a):id(i),t(tt),theta(a){}
};

struct MEdge{
    int from_id;
    int to_id;
    Eigen::Vector2d t;
    Eigen::Matrix3d covariance;
    Eigen::Vector3d from_pose;
    Eigen::Vector3d to_pose;
    double theta;

    MEdge(int f_id,int t_id,Eigen::Vector3d f_p,Eigen::Vector3d t_p, Eigen::Matrix3d c):\
   from_id(f_id),to_id(t_id),from_pose(f_p),to_pose(t_p),covariance(c){}
};

struct MPose{
    int id;
    Eigen::Vector2d t;
    Eigen::Matrix2d R;
    double theta;

    MPose(int i,Eigen::Vector2d tt,Eigen::Matrix2d r,double a):id(i),t(tt),R(r),theta(a){}
    MPose(){}
};

struct MFrame {
    MPose pose;
    RobotOdom odom;
    std::vector<Eigen::Vector2d> cloud;
    double info_k;
    double reject_k;
    double score;
    double time;

    MFrame(MPose p,RobotOdom o, std::vector<Eigen::Vector2d> c,double i, double r,double s,double t) :
            pose(p),odom(o),info_k(i), reject_k(r),score(s),time(s) {
        cloud.assign(c.begin(),c.end());
    }
    MFrame(){}
};

struct MSaveFrame{
    MPose pose;
    ros::Time ros_time;
    double run_time;
    sensor_msgs::LaserScan scan;

    MSaveFrame(MPose p,ros::Time t1,double t2):pose(p),ros_time(t1),run_time(t2){}
};


struct SubMap {
    bool lock;
    MFrame frame;

    SubMap(bool l, MFrame f) : lock(l), frame(f) {
    }
};

extern std::vector<RobotOdom> robot_odom_buf;
extern std::vector<MSaveFrame> saveframe_buf;
extern std::vector<MFrame> frame_buf;
extern std::vector<SubMap> submap_buf;
extern MPose loop_pose;

#endif //ICP_BNB_ICP_TYPE_H
