//
// Created by ray on 19-7-5.
//

#ifndef ICP_BNB_SCAN_PROCESS_H
#define ICP_BNB_SCAN_PROCESS_H
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <icp_define.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <sensor_msgs/LaserScan.h>
//#include <pcl/kdtree/kdtree_flann.h>
using namespace std;


vector<Eigen::Vector2d> scanToCloud(sensor_msgs::LaserScan scan,int voxel_r);
#endif //ICP_BNB_SCAN_PROCESS_H
