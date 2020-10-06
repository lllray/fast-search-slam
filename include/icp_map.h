//
// Created by ray on 19-8-21.
//

#ifndef ICP_BNB_ICP_MAP_H
#define ICP_BNB_ICP_MAP_H

#include <icp_type.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include "bits/stdc++.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "scan_process.h"
#include "icp_define.h"




struct BuildMapMessage{
    sensor_msgs::LaserScan::ConstPtr scan;
    MFrame frame;
    BuildMapMessage(sensor_msgs::LaserScan::ConstPtr s,MFrame &f):
            scan(s),frame(f){}
};

    extern vector<BuildMapMessage> buildMapMessage;
    extern int IcpMapUseID;

    extern cv::Mat IcpMap_0; //x*y
    extern cv::Mat IcpMap_1; //x*y


    cv::Point2i world_to_map(Eigen::Vector2d p_w, double resolution);
    void initMap();
    cv::Mat& getIcpMapUseing();
    cv::Mat& getIcpMapFreeing();
    void changeIcpMap();
    void updateMap(bool build_mode,cv::Mat& map,const std::vector<Eigen::Vector2d> pts_cloud,Eigen::Matrix2d &R, Eigen::Vector2d &t);
    void updateMap(bool build_mode,cv::Mat& map,int start_id,int end_id);
    void buildIcpMap(const sensor_msgs::LaserScan::ConstPtr& scan,MFrame &frame);
    void updateBuildMap(const sensor_msgs::LaserScan::ConstPtr& scan,MFrame &frame);


#endif //ICP_BNB_ICP_MAP_H
