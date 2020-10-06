//
// Created by ray on 19-7-2.
//
#include <ros/ros.h>
#ifndef LIDAR_SLAM_ICP_DEFINE_H
#define LIDAR_SLAM_ICP_DEFINE_H

#define PI 3.1415926

class Config{

public:
    Config(){}
    Config(ros::NodeHandle n){
        n.getParam ( "/icp_odometry/define/dataset_type", dataset_type );
        std::cout<<"dataset_type : "<<dataset_type<<std::endl;
        n.getParam ( "/icp_odometry/define/scan_size", scan_size );
        std::cout<<"scan_size : "<<scan_size<<std::endl;
        n.getParam ( "/icp_odometry/define/slam_model_mim", slam_model_mim );
            std::cout<<"slam_model_mim : "<<slam_model_mim<<std::endl;
        n.getParam ( "/icp_odometry/define/navigation_model_mim", navigation_model_mim );
            std::cout<<"navigation_model_mim : "<<navigation_model_mim<<std::endl;
        n.getParam ( "/icp_odometry/define/scan_max_distance", scan_max_distance );
            std::cout<<"scan_max_distance : "<<scan_max_distance<<std::endl;
        n.getParam ( "/icp_odometry/define/scan_min_distance", scan_min_distance );
            std::cout<<"scan_min_distance : "<<scan_min_distance<<std::endl;
        n.getParam ( "/icp_odometry/define/use_voxel_filter", use_voxel_filter );
            std::cout<<"use_voxel_filter : "<<use_voxel_filter<<std::endl;
        n.getParam ( "/icp_odometry/define/voxel_filter_size", voxel_filter_size );
            std::cout<<"voxel_filter_size : "<<voxel_filter_size<<std::endl;
        n.getParam ( "/icp_odometry/define/match_max_iters", match_max_iters );
            std::cout<<"match_max_iters : "<<match_max_iters<<std::endl;
        n.getParam ( "/icp_odometry/define/match_tree_m", match_tree_m );
            std::cout<<"match_tree_m : "<<match_tree_m<<std::endl;
        n.getParam ( "/icp_odometry/define/match_angel_max_step", match_angel_max_step );
            std::cout<<"match_angel_max_step : "<<match_angel_max_step<<std::endl;
        n.getParam ( "/icp_odometry/define/match_dist_max_step", match_dist_max_step );
            std::cout<<"match_dist_max_step : "<<match_dist_max_step<<std::endl;
        n.getParam ( "/icp_odometry/define/match_angel_max_range", match_angel_max_range );
            std::cout<<"match_angel_max_range : "<<match_angel_max_range<<std::endl;
        n.getParam ( "/icp_odometry/define/match_dist_max_range", match_dist_max_range );
            std::cout<<"match_dist_max_range : "<<match_dist_max_range<<std::endl;
        n.getParam ( "/icp_odometry/define/match_reject_point_max_k", match_reject_point_max_k );
            std::cout<<"match_reject_point_max_k : "<<match_reject_point_max_k<<std::endl;

        n.getParam ( "/icp_odometry/define/relocation_max_iters", relocation_max_iters );
        std::cout<<"relocation_max_iters : "<<relocation_max_iters<<std::endl;
        n.getParam ( "/icp_odometry/define/relocation_angel_max_step", relocation_angel_max_step );
            std::cout<<"relocation_angel_max_step : "<<relocation_angel_max_step<<std::endl;
        n.getParam ( "/icp_odometry/define/relocation_dist_max_step", relocation_dist_max_step );
            std::cout<<"relocation_dist_max_step : "<<relocation_dist_max_step<<std::endl;
        n.getParam ( "/icp_odometry/define/relocation_angel_max_range", relocation_angel_max_range );
            std::cout<<"relocation_angel_max_range : "<<relocation_angel_max_range<<std::endl;
        n.getParam ( "/icp_odometry/define/relocation_dist_max_range", relocation_dist_max_range );
            std::cout<<"relocation_dist_max_range : "<<relocation_dist_max_range<<std::endl;


        n.getParam ( "/icp_odometry/define/close_loop_max_iters", close_loop_max_iters );
        std::cout<<"close_loop_max_iters : "<<close_loop_max_iters<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_angel_max_step", close_loop_angel_max_step );
            std::cout<<"close_loop_angel_max_step : "<<close_loop_angel_max_step<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_dist_max_step", close_loop_dist_max_step );
            std::cout<<"close_loop_dist_max_step : "<<close_loop_dist_max_step<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_angel_max_range", close_loop_angel_max_range );
            std::cout<<"close_loop_angel_max_range : "<<close_loop_angel_max_range<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_dist_max_range", close_loop_dist_max_range );
            std::cout<<"close_loop_dist_max_range : "<<close_loop_dist_max_range<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_transnoise_max", close_loop_transnoise_max );
            std::cout<<"close_loop_transnoise_max : "<<close_loop_transnoise_max<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_rotnoise_max", close_loop_rotnoise_max );
            std::cout<<"close_loop_rotnoise_max : "<<close_loop_rotnoise_max<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_find_loop_max_distance", close_loop_find_loop_max_distance );
            std::cout<<"close_loop_find_loop_max_distance : "<<close_loop_find_loop_max_distance<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_find_loop_max_id_diff", close_loop_find_loop_max_id_diff );
            std::cout<<"close_loop_find_loop_max_id_diff : "<<close_loop_find_loop_max_id_diff<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_debug", close_loop_debug );
            std::cout<<"close_loop_debug : "<<close_loop_debug<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_score_k", close_loop_score_k );
            std::cout<<"close_loop_score_k : "<<close_loop_score_k<<std::endl;
        n.getParam ( "/icp_odometry/define/close_loop_reject_point_k", close_loop_reject_point_k );
            std::cout<<"close_loop_reject_point_k : "<<close_loop_reject_point_k<<std::endl;
        n.getParam ( "/icp_odometry/define/submap_max_distance", submap_max_distance );
            std::cout<<"submap_max_distance : "<<submap_max_distance<<std::endl;
        n.getParam ( "/icp_odometry/define/use_odom", use_odom );
        std::cout<<"use_odom : "<<use_odom<<std::endl;
        n.getParam ( "/icp_odometry/define/use_last_as_match_init", use_last_as_match_init );
            std::cout<<"use_last_as_match_init : "<<use_last_as_match_init<<std::endl;
        n.getParam ( "/icp_odometry/define/move_filter_min_dist", move_filter_min_dist );
            std::cout<<"move_filter_min_dist : "<<move_filter_min_dist<<std::endl;
        n.getParam ( "/icp_odometry/define/move_filter_min_angel", move_filter_min_angel );
            std::cout<<"move_filter_min_angel : "<<move_filter_min_angel<<std::endl;
        n.getParam ( "/icp_odometry/define/debug", debug );
            std::cout<<"debug : "<<debug<<std::endl;
        n.getParam ( "/icp_odometry/define/debug_info", debug_info );
            std::cout<<"debug_info : "<<debug_info<<std::endl;
        n.getParam ( "/icp_odometry/define/debug_image", debug_image );
            std::cout<<"debug_image : "<<debug_image<<std::endl;
        n.getParam ( "/icp_odometry/define/show_image_resolution", show_image_resolution );
            std::cout<<"show_image_resolution : "<<show_image_resolution<<std::endl;
        n.getParam ( "/icp_odometry/define/show_image_max_square", show_image_max_square );
            std::cout<<"show_image_max_square : "<<show_image_max_square<<std::endl;


        n.getParam ( "/icp_odometry/define/dataset_x_offset", dataset_x_offset );
        std::cout<<"dataset_x_offset : "<<dataset_x_offset<<std::endl;
        n.getParam ( "/icp_odometry/define/dataset_y_offset", dataset_y_offset );
        std::cout<<"dataset_y_offset : "<<dataset_y_offset<<std::endl;
        n.getParam ( "/icp_odometry/define/dataset_theta_offset", dataset_theta_offset );
        std::cout<<"dataset_theta_offset : "<<dataset_theta_offset<<std::endl;

        n.getParam ( "/icp_odometry/define/init_x_offset", init_x_offset );
        std::cout<<"init_x_offset : "<<init_x_offset<<std::endl;
        n.getParam ( "/icp_odometry/define/init_y_offset", init_y_offset );
        std::cout<<"init_y_offset : "<<init_y_offset<<std::endl;

        n.getParam ( "/icp_odometry/define/icpmap_wigth", icpmap_wigth );
            std::cout<<"icpmap_wigth : "<<icpmap_wigth<<std::endl;
        n.getParam ( "/icp_odometry/define/icpmap_high", icpmap_high );
            std::cout<<"icpmap_high : "<<icpmap_high<<std::endl;
        n.getParam ( "/icp_odometry/define/icpmap_resolution", icpmap_resolution );
            std::cout<<"icpmap_resolution : "<<icpmap_resolution<<std::endl;
        n.getParam ( "/icp_odometry/define/icpmap_point_add_range", icpmap_point_add_range );
            std::cout<<"icpmap_point_add_range : "<<icpmap_point_add_range<<std::endl;
        n.getParam ( "/icp_odometry/define/icpmap_show_k", icpmap_show_k );
            std::cout<<"icpmap_show_k : "<<icpmap_show_k<<std::endl;
//        n.getParam ( "/icp_odometry/define/icpmax_value_max", icpmax_value_max );
        icpmax_value_max=icpmap_point_add_range*icpmap_show_k;
            std::cout<<"icpmax_value_max : "<<icpmax_value_max<<std::endl;
//        n.getParam ( "/icp_odometry/define/icpmap_bad_point_k", icpmap_bad_point_k );
        n.getParam ( "/icp_odometry/define/icpmap_bad_k", icpmap_bad_k );
        std::cout<<"icpmap_bad_k : "<<icpmap_bad_k<<std::endl;

        icpmap_bad_point_value=icpmap_show_k*icpmap_bad_k*icpmap_point_add_range;
            std::cout<<"icpmap_bad_point_value : "<<icpmap_bad_point_value<<std::endl;
        n.getParam ( "/icp_odometry/define/icpmap_core_point_range", icpmap_core_point_range );
            std::cout<<"icpmap_core_point_range : "<<icpmap_core_point_range<<std::endl;
        n.getParam ( "/icp_odometry/define/icpmap_point_clear_range", icpmap_point_clear_range );
            std::cout<<"icpmap_point_clear_range : "<<icpmap_point_clear_range<<std::endl;

        n.getParam ( "/icp_odometry/define/icpmap_hit_k", icpmap_hit_k );
        std::cout<<"icpmap_hit_k : "<<icpmap_hit_k<<std::endl;


        match_range_k=1;
        match_step_k=1;
    }
//#my dataset 3 ->1
//#my dataset 4 ->2
//#aces_logfile ->3
//#intel_logfile->4
//#FREIBURGINDOOR->5
    int DATASET_TYPE_(){return dataset_type;}
    int SCAN_SIZE_(){return scan_size;}
    bool SLAM_MODEL_MIM_(){return slam_model_mim;}
    bool NAVIGATION_MODEL_MIM_(){return navigation_model_mim;}
    double SCAN_MAX_DISTANCE_(){return scan_max_distance;}
    double SCAN_MIN_DISTANCE_(){return scan_min_distance;}

    bool USE_VOXEL_FILTER_(){return use_voxel_filter;}
    double VOXEL_FILTER_SIZE_(){return voxel_filter_size;}

    int MATCH_MAX_ITERS_(){return match_max_iters;}
    int MATCH_TREE_M_(){return match_tree_m;}
    int MATCH_ANGEL_MAX_STEP_(){return match_angel_max_step*match_step_k;}
    int MATCH_DIST_MAX_STEP_(){return match_dist_max_step*match_step_k;}
    double MATCH_ANGEL_MAX_RANGE_(){return match_angel_max_range*match_range_k;}
    double MATCH_DIST_MAX_RANGE_(){return match_dist_max_range*match_range_k;}
    double MATCH_REJECT_POINT_MAX_K_(){return match_reject_point_max_k;}

    int RELOCATION_MAX_ITERS_(){return relocation_max_iters;}
    int RELOCATION_ANGEL_MAX_STEP_(){return relocation_angel_max_step;}
    int RELOCATION_DIST_MAX_STEP_(){return relocation_dist_max_step;}
    double RELOCATION_ANGEL_MAX_RANGE_(){return relocation_angel_max_range;}
    double RELOCATION_DIST_MAX_RANGE_(){return relocation_dist_max_range;}


    int CLOSE_LOOP_MAX_ITERS_(){return close_loop_max_iters;}
    int CLOSE_LOOP_ANGEL_MAX_STEP_(){return close_loop_angel_max_step;}
    int CLOSE_LOOP_DIST_MAX_STEP_(){return close_loop_dist_max_step;}
    double CLOSE_LOOP_ANGEL_MAX_RANGE_(){return close_loop_angel_max_range;}
    double CLOSE_LOOP_DIST_MAX_RANGE_(){return close_loop_dist_max_range;}

    double CLOSE_LOOP_TRANSNOISE_MAX_(){return close_loop_transnoise_max;}
    double CLOSE_LOOP_ROTNOISE_MAX_(){return close_loop_rotnoise_max;}
    double CLOSE_LOOP_FIND_LOOP_MAX_DISTANCE_(){return close_loop_find_loop_max_distance;}
    double CLOSE_LOOP_FIND_LOOP_MAX_ID_DIFF_(){return close_loop_find_loop_max_id_diff;}
    bool CLOSE_LOOP_DEBUG_(){return close_loop_debug;}

    double CLOSE_LOOP_SCORE_K_(){return close_loop_score_k;}
    double CLOSE_LOOP_REJECT_POINT_K_(){return close_loop_reject_point_k;}

    double SUBMAP_MAX_DISTANCE_(){return submap_max_distance;}

    bool USE_ODOM_(){return use_odom;}
    bool USE_LAST_AS_MATCH_INIT_(){return use_last_as_match_init;}
    bool USE_MOVE_FILTER_(){return use_move_filter;}
    double MOVE_FILTER_MIN_DIST_(){return move_filter_min_dist;}
    double MOVE_FILTER_MIN_ANGEL_(){return move_filter_min_angel;}

    bool DEBUG_(){return debug;}
    bool DEBUG_INFO_(){return debug_info;}
    bool DEBUG_IMAGE_(){return debug_image;}

    double SHOW_IMAGE_RESOLUTION_(){return show_image_resolution;}
    int SHOW_IMAGE_MAX_SQUARE_(){return show_image_max_square;}

    double DATASET_X_OFFSET_(){return dataset_x_offset;}
    double DATASET_Y_OFFSET_(){return dataset_y_offset;}
    double DATASET_THETA_OFFSET_(){return dataset_theta_offset;}

    double INIT_X_OFFSET_(){return init_x_offset;}
    double INIT_Y_OFFSET_(){return init_y_offset;}
    int ICPMAP_WIGTH_(){return icpmap_wigth;}
    int ICPMAP_HIGH_(){return icpmap_high;}
    double ICPMAP_RESOLUTION_(){return icpmap_resolution;}
    int ICPMAP_POINT_ADD_RANGE_(){return icpmap_point_add_range;}
    int ICPMAP_SHOW_K_(){return icpmap_show_k;}
    int ICPMAP_VALUE_MAX_(){return icpmax_value_max;}
    int ICPMAP_BAD_POINT_VALUE_(){return icpmap_bad_point_value;}
    int ICPMAP_CORE_POINT_RANGE_(){return icpmap_core_point_range;}
    int ICPMAP_POINT_CLEAR_RANGE_(){return icpmap_point_clear_range;}
    double ICPMAP_HIT_K_(){return icpmap_hit_k;}
    void SET_MATCH_RANGE_K_(int k){
        match_range_k=k;
    };
    void SET_MATCH_STEP_K_(int k){
        match_step_k=k;
    };

private:
    int scan_size;
    int dataset_type;

    bool slam_model_mim;
    bool navigation_model_mim;

    double scan_max_distance;
    double scan_min_distance;

    bool use_voxel_filter;
    double voxel_filter_size;

    int match_step_k;
    int match_range_k;
    int match_max_iters;
    int match_tree_m;
    int match_angel_max_step;
    int match_dist_max_step;
    double match_angel_max_range;
    double match_dist_max_range;

    double match_reject_point_max_k;

    int relocation_max_iters;
    int relocation_angel_max_step;
    int relocation_dist_max_step;
    double relocation_angel_max_range;
    double relocation_dist_max_range;

    int close_loop_max_iters;
    int close_loop_angel_max_step;
    int close_loop_dist_max_step;
    double close_loop_angel_max_range;
    double close_loop_dist_max_range;

    double close_loop_transnoise_max;
    double close_loop_rotnoise_max;
    double close_loop_find_loop_max_distance;
    double close_loop_find_loop_max_id_diff;
    bool close_loop_debug;

    double close_loop_score_k;
    double close_loop_reject_point_k;

    double submap_max_distance;
    bool use_odom;
    bool use_last_as_match_init;
    bool use_move_filter;
    double move_filter_min_dist;
    double move_filter_min_angel;

    bool debug;
    bool debug_info;
    bool debug_image;

    double show_image_resolution;
    int show_image_max_square;

    double init_x_offset;
    double init_y_offset;
    double dataset_x_offset;
    double dataset_y_offset;
    double dataset_theta_offset;
    int icpmap_wigth;
    int icpmap_high;
    double icpmap_resolution;
    int icpmap_point_add_range;
    int icpmap_show_k;
    int icpmax_value_max;
    int icpmap_bad_point_value;
    int icpmap_core_point_range;
    int icpmap_point_clear_range;
    double icpmap_hit_k;
    int icpmap_bad_k;


};

extern Config config;
extern pthread_mutex_t update_map_mutex;
extern pthread_mutex_t map_mutex;
extern pthread_mutex_t get_init_pose_mutex;
extern bool start_match;
extern bool relocalization;
extern int update_map_state; // 0 none / 1 update cur map / 2 update cur pose / 3 update all
extern int update_map_id;
extern double g_bnb_rate[3];
extern double g_qf_rate[3][2];
#endif //LIDAR_SLAM_ICP_DEFINE_H
