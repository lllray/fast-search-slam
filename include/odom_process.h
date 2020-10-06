//
// Created by ray on 19-10-24.
//

#ifndef ICP_BNB_ODOM_PROCESS_H
#define ICP_BNB_ODOM_PROCESS_H


#include "icp_type.h"
#include "icp_define.h"
#include "tools.h"
void getInitPoseFromOdom(MFrame &frame,Eigen::Matrix2d &R,Eigen::Vector2d &t,Eigen::Matrix2d map_R,Eigen::Vector2d map_t);


#endif //ICP_BNB_ODOM_PROCESS_H