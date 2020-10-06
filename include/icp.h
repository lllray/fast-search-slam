//
// Created by ray on 19-7-5.
//

#ifndef ICP_H
#define ICP_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include "bits/stdc++.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <icp_type.h>
#include "tools.h"
#include "icp_map.h"
#include <icp_define.h>

//namespace LIDAR
//{

	struct Queue{
		int n;
		double squared_dist_Estimate;
	};
	struct cmp{
		bool operator() (const Queue s1, const Queue s2) {
			//小的在前 //大的在前
			return s1.squared_dist_Estimate > s2.squared_dist_Estimate;
		}
		};
	struct State{
	    double x;
	    double y;
	    double a;
		int bnb_n;
        std::vector<int> rp_buf;
		double sum_squared_dist;
		double score;
		Eigen::Matrix2d R;
		Eigen::Vector2d t;
	};
	enum MatchState{
		normal=1,
		relocation,
		close_loop_check
	};
class ICP{
public:
	ICP();

public:
//	static bool Match(
//		const std::vector<Eigen::Vector2d> pts_cloud,
//		double n_iters, double epsilon, double min_err,
//		Eigen::Matrix2d& R, Eigen::Vector2d& t,Eigen::Matrix2d& l_R,Eigen::Vector2d& l_t,
//		double &score,const cv::Mat &icp_map,MatchState state
//	);
	static bool Match(
			MFrame &frame,
			Eigen::Matrix2d& R, Eigen::Vector2d& t,Eigen::Matrix2d& l_R,Eigen::Vector2d& l_t,
			const cv::Mat &icp_map,MatchState state
	);

private:
	static int icp_match_id;
	static double reject_k_average;
	static double best_score_average;

	
}; //class ICP
	
	
//} // namespace LIDAR

#endif
