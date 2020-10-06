//
// Created by ray on 19-9-17.
//

#ifndef ICP_BNB_TOOLS_H
#define ICP_BNB_TOOLS_H

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <vector>
#include <math.h>
#include <fstream>
#include <iostream>

#include "icp_type.h"
#include "icp_define.h"

using namespace std;
using namespace cv;

Mat oneCloudToMat(vector<Eigen::Vector2d> cloud,int mat_size,double resolution);
Mat twoCloudToMat(vector<Eigen::Vector2d> cloud1,vector<Eigen::Vector2d> cloud2,int mat_size,double resolution);
double rToTheta(const Eigen::Matrix2d& R);
Eigen::Matrix2d thetaToR(double theta);
vector<Eigen::Vector2d> cloudFilter(const vector<Eigen::Vector2d> &pts_cloud);
void writePoseFile(ofstream &file,MSaveFrame frame);
void writeBenchMarkFile(ofstream &file,MSaveFrame frame) ;


#endif //ICP_BNB_TOOLS_H