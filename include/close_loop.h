//
// Created by ray on 19-8-30.
//

#ifndef ICP_BNB_CLOSE_LOOP_H
#define ICP_BNB_CLOSE_LOOP_H

#include "icp_type.h"
#include "icp.h"
#include "icp_define.h"
#include "icp_map.h"
#include "tools.h"
#include "g2o_solver.h"


int get_loop_id(int cur_id,Eigen::Matrix2d &R, Eigen::Vector2d &t);
bool optimize(int cur_id,int loop_id, Eigen::Vector2d &t,double theta);
extern int last_loop_id;

#endif //ICP_BNB_CLOOSE_LOOP_H
