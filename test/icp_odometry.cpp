#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <pthread.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

#include <icp.h>
#include <icp_map.h>
#include <icp_define.h>
#include <scan_process.h>
#include "odom_process.h"
#include "close_loop.h"
#include "tools.h"
#include "icp_type.h"
using namespace std;
using namespace cv;
double g_bnb_rate[3]={0.0,0.0,0.0};
double g_qf_rate[3][2]={0.0,0.0,0.0,0.0,0.0,0.0};


bool start_match=false;
bool relocalization=false;
int update_map_state=0;
int update_map_id=0;

pthread_mutex_t update_map_mutex;
pthread_mutex_t map_mutex;
pthread_mutex_t get_init_pose_mutex;


Eigen::Matrix2d init_R = Eigen::Matrix2d::Identity();
Eigen::Vector2d init_t(0.0, 0.0);

ros::Subscriber scan_sub_;
ros::Subscriber odom_sub_;
ros::Publisher g_odom_pub;
ros::Publisher icp_map_pub;

boost::shared_ptr<tf::TransformBroadcaster> robot_odom_broadcaster;
boost::shared_ptr<tf::TransformBroadcaster> slam_odom_broadcaster;
boost::shared_ptr<tf::TransformBroadcaster> odom_broadcaster;
boost::shared_ptr<tf::TransformBroadcaster> laser_broadcaster;



bool odometryPub(const double& theta,const Eigen::Vector2d& t) {
    static clock_t last_time=clock();
    static Eigen::Vector2d last_t=t;

    static double last_theta=theta;

    double cur_time=clock();
    double delta_time=(double)(cur_time-last_time)/CLOCKS_PER_SEC;

    if(delta_time<0.001) {
        ROS_INFO("[odometryPub] delta_time too short is =%f",delta_time);
        return false;
    }
    double delta_x=t[0]-last_t[0];
    double delta_y=t[1]-last_t[1];
    double delta_theta=theta-last_theta;
    double vx=delta_x/delta_time;
    double vy=delta_y/delta_time;
    double omega=delta_theta/delta_time;


    tf::Quaternion q;
    q = tf::createQuaternionFromYaw(0);
    odom_broadcaster->sendTransform(
            tf::StampedTransform(
                    tf::Transform(q, tf::Vector3(0.0, 0.0, 0.0)),
                    ros::Time::now(),"map", "slam_odom"));

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "slam_odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = t[0];
    odom_trans.transform.translation.y = t[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    slam_odom_broadcaster->sendTransform(odom_trans);

    //publish odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "slam_odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = t[0];
    odom.pose.pose.position.y = t[1];
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.angular.z = omega;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    g_odom_pub.publish(odom);

    //tf::Quaternion q;
    //q = tf::createQuaternionFromYaw(0);
    laser_broadcaster->sendTransform(
            tf::StampedTransform(
                    tf::Transform(q, tf::Vector3(0.0, 0.0, 0.0)),
                    ros::Time::now(),"base_link", "base_scan"));
    if(config.USE_MOVE_FILTER_()&&(fabs(delta_x)<config.MOVE_FILTER_MIN_DIST_()&&fabs(delta_y)<config.MOVE_FILTER_MIN_DIST_()&&fabs(omega)<config.MOVE_FILTER_MIN_ANGEL_()))
    {
        //ROS_INFO("[move filter] move is too tiny");
        return false;
    }
    last_time=cur_time;
    last_theta=theta;
    last_t=t;
    return true;
}

void icpMapPub(const std::string& frame_id,nav_msgs::OccupancyGrid& map){
    map.header.frame_id = frame_id;
    map.header.stamp = ros::Time::now();

    map.info.width = config.ICPMAP_WIGTH_();
    map.info.height = config.ICPMAP_HIGH_();
    map.info.resolution = config.ICPMAP_RESOLUTION_();
    map.info.origin.position.x = -config.ICPMAP_WIGTH_()*config.ICPMAP_RESOLUTION_()*0.5;
    map.info.origin.position.y = -config.ICPMAP_HIGH_()*config.ICPMAP_RESOLUTION_()*0.5;

    const int N = config.ICPMAP_WIGTH_() * config.ICPMAP_HIGH_();
    Mat pub_map=getIcpMapUseing();
    for(size_t i= 0; i < N; i ++)
    {
        unsigned char value=config.ICPMAP_VALUE_MAX_()-*(pub_map.data+i);
       // if(*(IcpMap.data+i)>100)
//        if(value==config.ICPMAP_VALUE_MAX_()||value<=(config.ICPMAP_VALUE_MAX_()-config.ICPMAP_SHOW_K_()*(config.ICPMAP_CORE_POINT_RANGE_()*4)))value=0;
//        map.data.emplace_back((unsigned char)(value*0.5));
        if(value==config.ICPMAP_VALUE_MAX_())value=0;
        map.data.emplace_back((unsigned char)(value*0.5));

    }
}
int time_num=0;
double time_average=0.0;
double time_sum=0.0;
void saveMap(const std::string& img_dir,const std::string& icp_img_dir, const std::string& cfg_dir){
    /* save icp map */
    if(imwrite(icp_img_dir, getIcpMapUseing())) cout<<"[save] save icp_map success!"<<endl;
    else  cout<<"[save] save icp_map fail!"<<endl;

    /* save navigation map */
    Mat navigation_map(Size(config.ICPMAP_WIGTH_(), config.ICPMAP_HIGH_()), CV_8UC1, Scalar(0)); //x*y
    const int N = config.ICPMAP_WIGTH_() * config.ICPMAP_HIGH_();
    unsigned char *pPixel = 0;
    Mat save_map=getIcpMapUseing();
    for(size_t i= 0; i < N; i ++)
    {
        unsigned char value=config.ICPMAP_VALUE_MAX_()-*(save_map.data+i);
        if(value>=config.ICPMAP_VALUE_MAX_()||value<(config.ICPMAP_VALUE_MAX_()-config.ICPMAP_SHOW_K_()*(config.ICPMAP_CORE_POINT_RANGE_())))value=0;
        pPixel = navigation_map.data + i;
        *pPixel=(unsigned char)(value*0.5);
    }
    if(imwrite(img_dir, navigation_map)) cout<<"[save] save navigation_map success!"<<endl;
    else  cout<<"[save] save navigation_map fail!"<<endl;

    /* save param and result data */
    time_average=time_sum/time_num;
    g_bnb_rate[0]= g_bnb_rate[0]/time_num;
    g_bnb_rate[1]= g_bnb_rate[1]/time_num;
    g_bnb_rate[2]= g_bnb_rate[2]/time_num;
    g_qf_rate[0][0]= g_qf_rate[0][0]/time_num;
    g_qf_rate[1][0]= g_qf_rate[1][0]/time_num;
    g_qf_rate[2][0]= g_qf_rate[2][0]/time_num;

    /* 保存配置 */
    std::ofstream  file;
    file.open(cfg_dir);
    file << "map:"<< std::endl
         << "  size_x: " << config.ICPMAP_WIGTH_() << std::endl
         << "  size_y: " << config.ICPMAP_HIGH_() << std::endl
         << "  init_x: " << config.ICPMAP_WIGTH_()/2 << std::endl
         << "  init_y: " << config.ICPMAP_HIGH_()/2 << std::endl
         << "  cell_size: " << config.ICPMAP_RESOLUTION_() << std::endl
         << "  time_num: " << time_num << std::endl
         << "  time_sum: " << time_sum << std::endl
         << "  time_average: " << time_average << std::endl
         << "  g_bnb_rate[0]: " << g_bnb_rate[0] << std::endl
         << "  g_bnb_rate[1]: " << g_bnb_rate[1] << std::endl
         << "  g_bnb_rate[2]: " << g_bnb_rate[2] << std::endl
         << "  g_qf_rate[0][0]: " << g_qf_rate[0][0] << std::endl
         << "  g_qf_rate[1][0]: " << g_qf_rate[1][0] << std::endl
         << "  g_qf_rate[2][0]: " << g_qf_rate[2][0] << std::endl;
}
Eigen::Vector3d robot_odom(0.0,0.0,0.0);


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    double theta = yaw;
    RobotOdom odom_temp;
    odom_temp.pose[0] = odom->pose.pose.position.x;
    odom_temp.pose[1] = odom->pose.pose.position.y;
    odom_temp.pose[2] = theta;
    odom_temp.time = odom->header.stamp.toSec();

    //ROS_INFO("[odom] recevie odom time=%f x=%f y=%f theta=%f",odom_temp.time,odom->pose.pose.position.x,odom->pose.pose.position.y,theta);

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_temp.pose[2]);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "odom";

    odom_trans.transform.translation.x = odom_temp.pose[0];
    odom_trans.transform.translation.y = odom_temp.pose[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    robot_odom_broadcaster->sendTransform(odom_trans);

    pthread_mutex_lock(&get_init_pose_mutex);
    while ((robot_odom_buf.size() > 2) && ((odom_temp.time - robot_odom_buf[0].time) > 1.0)) {
        //ROS_INFO("[odom] delete odom time=%f",robot_odom_buf[0].time);
        robot_odom_buf.erase(robot_odom_buf.begin());
    }
    robot_odom_buf.push_back(odom_temp);
    pthread_mutex_unlock(&get_init_pose_mutex);
}

Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
Eigen::Vector2d t(0.0, 0.0);

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan){

    if((update_map_state==2)&&config.CLOSE_LOOP_DEBUG_())return;

    static ros::Time ros_start_time = scan->header.stamp;
    static double match_info=0.0;
    static int interval_num=0;
    ros::Time ros_cur_time=scan->header.stamp;

    std::vector<Eigen::Vector2d> pts_cloud;
    pts_cloud.reserve (scan->ranges.size());
    pts_cloud.clear();
    pts_cloud=scanToCloud(*scan,1);

    if(config.DEBUG_INFO_())ROS_INFO("[scan callback] input cloud size is =%d",pts_cloud.size());

    MFrame frame;
    frame.cloud=pts_cloud;
    frame.time=ros_cur_time.toSec();


    //使用odom做先验
    if(config.USE_ODOM_()){
        R = Eigen::Matrix2d::Identity();
        t[0]=0.0;
        t[1]=0.0;
        getInitPoseFromOdom(frame,R,t,init_R,init_t);
    }else if(config.USE_LAST_AS_MATCH_INIT_()) {
    // do nothing
    }else{
        R = Eigen::Matrix2d::Identity();
        t[0]=0.0;
        t[1]=0.0;
    }


    if((config.NAVIGATION_MODEL_MIM_()||config.SLAM_MODEL_MIM_())&&start_match&&(pts_cloud.size()>0)) {

        //icp match

        clock_t start,match_finish,build_finish;
        double totaltime;
        start=clock();
        double cur_score;
        if(!relocalization&&ICP::Match(frame, R, t,init_R,init_t,getIcpMapUseing(),normal)) {
            match_finish=clock();
            totaltime=(double)(match_finish-start)/CLOCKS_PER_SEC;
            if(config.DEBUG_INFO_())std::cout<<"\n|*******************************匹配的运行时间为"<<totaltime<<"秒！"<<std::endl;


            time_num++;
            time_sum+=totaltime;

            init_t+=t;
            init_R=R*init_R;

            if(update_map_state==1){
                ROS_INFO("[update] start update loop pose");

                init_t=loop_pose.t+init_t-frame_buf[loop_pose.id].pose.t;
                init_R=frame_buf[loop_pose.id].pose.R.inverse()*init_R*loop_pose.R;
                ROS_INFO("[update] finish update loop pose");
                update_map_id=frame_buf.size()-1;
                update_map_state=2;
            }


            double theta=rToTheta(init_R);
            if(config.DEBUG_INFO_())ROS_INFO("t.x= %f  t.y= %f , theta =%f",t[0],t[1],rToTheta(R));
            if(config.DEBUG_INFO_())ROS_INFO("t.x= %f  t.y= %f , theta =%f",init_t[0],init_t[1],theta);

            //pub odom and motion filter
            //ROS_INFO("cur_score=%f",cur_score);
            match_info+=frame.info_k;
            interval_num++;


            if(odometryPub(theta,init_t)) {
                //ROS_INFO("this score=%f",match_score/=interval_num);
                if(config.DEBUG_INFO_())ROS_INFO("START updateBuildMap");
                match_info/=interval_num;

                frame.info_k=match_info;
                frame.pose.R=init_R;
                frame.pose.t=init_t;
                frame.pose.theta=theta;
                updateBuildMap(scan, frame);
                match_info=0;
                interval_num=0;
                //buildIcpMap(scan, pts_cloud, init_R, init_t);
            }

            MSaveFrame write_frame(frame.pose,ros_cur_time,ros_cur_time.toSec()-ros_start_time.toSec());
            write_frame.scan=*scan;
            ofstream benchmark_file("/home/ray/data_output.txt", ios::app);
            ofstream pose_file("/home/ray/pose_output.txt", ios::app);
            writeBenchMarkFile(benchmark_file,write_frame);
            writePoseFile(pose_file,write_frame);
        }else{
            ROS_ERROR("get pose fail!!!!!  START relocalization");
            relocalization=true;
            //double cur_score;
            if (ICP::Match(frame, R, t,init_R,init_t,getIcpMapUseing(),relocation)) {
                init_t += t;
                init_R = R * init_R;
                R = Eigen::Matrix2d::Identity();
                t[0] = 0.0;
                t[1] = 0.0;
                double theta = rToTheta(init_R);
                match_info+=frame.info_k;
                interval_num++;
                frame.info_k=match_info;
                frame.pose.R=init_R;
                frame.pose.t=init_t;
                frame.pose.theta=theta;
                if(odometryPub(theta,init_t)) {

                    updateBuildMap(scan,frame);
                    match_info=0;
                    interval_num=0;
                }
                MSaveFrame write_frame(frame.pose,ros_cur_time,ros_cur_time.toSec()-ros_start_time.toSec());
                write_frame.scan=*scan;
                if(config.DEBUG_INFO_())ROS_INFO("t.x= %f  t.y= %f , theta =%f", init_t[0], init_t[1], theta);
                ofstream benchmark_file("/home/ray/data_output.txt", ios::app);
                ofstream pose_file("/home/ray/pose_output.txt", ios::app);
                writeBenchMarkFile(benchmark_file,write_frame);
                writePoseFile(pose_file,write_frame);
                ROS_ERROR("relocalization success!!!");
                relocalization=false;
            }else{
                //重定位失败，增加范围
            }
        }
    }else{
            if (config.SLAM_MODEL_MIM_() && pts_cloud.size() > 0) {


                //update_last_time(ros_cur_time.toSec());

                frame.info_k=match_info;
                frame.pose.R=init_R;
                frame.pose.t=init_t;
                double theta = rToTheta(init_R);
                frame.pose.theta=theta;
                buildIcpMap(scan, frame);

                start_match = true;
            } else if (config.NAVIGATION_MODEL_MIM_() && pts_cloud.size() > 0) {
                double cur_score;
                if (ICP::Match(frame, R, t,init_R,init_t,getIcpMapUseing(),relocation)) {
                    init_t += t;
                    init_R = R * init_R;
                    R = Eigen::Matrix2d::Identity();
                    t[0] = 0.0;
                    t[1] = 0.0;
                    double theta = rToTheta(init_R);

                    if(config.DEBUG_INFO_())ROS_INFO("t.x= %f  t.y= %f , theta =%f", init_t[0], init_t[1], theta);
                    start_match = true;
//                imshow("IcpMap",IcpMap);
//                waitKey(0);
                } else {
                    start_match = false;
                }
                //重定位
            }

    }

    if (config.DEBUG_IMAGE_()) {
        oneCloudToMat(pts_cloud,config.SHOW_IMAGE_MAX_SQUARE_(),config.SHOW_IMAGE_RESOLUTION_());
    }
}

void threadMapPubFunc(){
    ros::NodeHandle n_map_pub;
    ros::Rate r(0.1);
    while(n_map_pub.ok()) {
        if (config.DEBUG_() && start_match) {
            if(config.DEBUG_INFO_())ROS_INFO("start pub map");
            nav_msgs::OccupancyGrid icp_map;
            icpMapPub("map", icp_map);
            icp_map_pub.publish(icp_map);
            if(config.DEBUG_INFO_())ROS_INFO("finsh pub map");
        }
        ros::spinOnce();
        r.sleep();
    }
}
void threadMapBuildFunc(){
    ros::NodeHandle n_map_build;
    ros::Rate r(100);
    while(n_map_build.ok()) {
        //odometryPub(0,init_t);
        if (buildMapMessage.size()>0) {

            BuildMapMessage recv_container=buildMapMessage.front();
            buildIcpMap(recv_container.scan,recv_container.frame );
            pthread_mutex_lock(&update_map_mutex);  //加锁
            buildMapMessage.erase(buildMapMessage.begin());
            pthread_mutex_unlock(&update_map_mutex); //解锁


        }
        ros::spinOnce();
        r.sleep();
    }
}

int last_loop_id=0;

void threadCloseLoopFunc(){
    ros::NodeHandle n_close_loop;
    ros::Rate r(100);
    int next_id=1;
    while(n_close_loop.ok()) {
//        Eigen::Vector2d t_loop(0,0);
//        optimize(0,0,t_loop,0);
        //如果已完成子图id+1小于目前已有子图数量，则开始闭环检测，
        // 即若完成子图闭环检测数量0，目前子图数量为2，则开始闭环检测,数量为１时该子图尚未构建完成
        //ROS_INFO("submap_buf.size %d",submap_buf.size());
        if ((submap_buf.size()>next_id)&&config.DEBUG_()) {

            //如果第一个子图未闭环检测
            if(submap_buf[next_id].lock){
                next_id++;
            }else{
                vector<Eigen::Vector2d> pts_cloud=submap_buf[next_id].frame.cloud;
                Eigen::Matrix2d R_temp = Eigen::Matrix2d::Identity();
                Eigen::Vector2d t_temp(0.0, 0.0);
                Eigen::Matrix2d submap_R=submap_buf[next_id].frame.pose.R;
                Eigen::Vector2d submap_t=submap_buf[next_id].frame.pose.t;
                //ROS_INFO("[loop] submap_buf f_frame_id is %d",submap_buf[0].frame_id-submap_buf[0].sub_id);
                ROS_INFO("[loop] submap_buf frame_id is %d",submap_buf[next_id].frame.pose.id);
                //ROS_INFO("[loop] frame_buf size is %d",frame_buf.size());
                ROS_INFO("START COLSE LOOP CHECK -----------------------------------------");
               // double cur_score;
               MFrame frame=submap_buf[next_id].frame;
                if(ICP::Match(frame, R_temp, t_temp,submap_R,submap_t,getIcpMapUseing(),close_loop_check)){
                    cout << "submap_t=" << submap_t << endl;
                    cout << "t_temp=" << t_temp << endl;
                    ROS_INFO("[loop] cur score=%f  old score=%f",frame.score,submap_buf[next_id].frame.score);
                    ROS_INFO("[loop] cur reject=%f  old reject=%f",frame.reject_k,submap_buf[next_id].frame.reject_k);
                    if (frame.score*0.9 < submap_buf[next_id].frame.score && frame.reject_k<=submap_buf[next_id].frame.reject_k) {
                        ROS_ERROR("[loop] find loop!!!!!!!!!------------------------------------------");
                        //开始优化
                        int cur_id = frame.pose.id;
                        // int f_frame_id = submap_buf[0].frame_id - submap_buf[0].sub_id;
                        Eigen::Matrix2d R_loop = R_temp * submap_R;
                        Eigen::Vector2d t_loop = submap_t + t_temp;
                        ROS_INFO("[loop] t_loop is (%f,%f)", t_loop[0], t_loop[1]);
                        double theta = rToTheta(R_loop);
                        int loop_id = get_loop_id(cur_id, R_loop, t_loop);
                        ROS_INFO("[loop] cur_id is (%d)", cur_id);
                        ROS_INFO("[loop] loop_id is (%d)", loop_id);
                        if ((loop_id > 0 && loop_id <= cur_id) &&
                            (cur_id - loop_id > config.CLOSE_LOOP_FIND_LOOP_MAX_ID_DIFF_())) {
                            if (optimize(loop_id, cur_id, t_loop, theta)) {

                                //保存闭环结果
                                if (config.CLOSE_LOOP_DEBUG_()) {
                                //if (cur_id>5000){
                                    imwrite("/home/ray/my_map/old_map.png", getIcpMapFreeing());
                                    //imshow("1", getIcpMapFreeing());
                                    //imshow("2", getIcpMapUseing());
                                    waitKey(0);
                                }
                            }
                            //last_loop_id = submap_buf[0].frame_id;
                        } else {
                            ROS_ERROR("[loop error] loop id is error or too close!");
                        }
                    }else{

                        ROS_ERROR("[loop error] not better loop!!!!!!!!!");
                    }
                }else ROS_ERROR("[loop error] not find loop!!!!!!!!!");

                    submap_buf[next_id].lock=true;
            }
        }
        ros::spinOnce();
        r.sleep();
    }
}
Config config;

int main(int argc, char** argv){

    ros::init(argc, argv, "icp_test");
    ros::NodeHandle n;

    //读取路径等参数
    std::string map_image_save_dir, map_config_save_dir,icp_map_image_save_dir;
    n.getParam ( "/icp_odometry/map_image_save_dir", map_image_save_dir );
    n.getParam ( "/icp_odometry/icp_map_image_save_dir", icp_map_image_save_dir );
    n.getParam ( "/icp_odometry/map_config_save_dir", map_config_save_dir );

    //读取程序所有参数
    config=*(new Config(n));

    //初始化地图
    initMap();

    //初始化初始位姿
    init_t[0]=config.INIT_X_OFFSET_();
    init_t[1]=config.INIT_Y_OFFSET_();

    //导航模式，读取全局 icp map
    if(config.NAVIGATION_MODEL_MIM_()){
        Mat img=imread(icp_map_image_save_dir,2 | 4);
        if(img.empty()){
            cout<<"Can not load icp map."<<endl;
            return -1;
        }
        getIcpMapUseing()=img;
    }


    pthread_mutex_init(&update_map_mutex,NULL);
    pthread_mutex_init(&map_mutex,NULL);
    pthread_mutex_init(&get_init_pose_mutex,NULL);

    robot_odom_broadcaster.reset(new tf::TransformBroadcaster());
    slam_odom_broadcaster.reset(new tf::TransformBroadcaster());
    laser_broadcaster.reset(new tf::TransformBroadcaster());
    odom_broadcaster.reset(new tf::TransformBroadcaster());

    odom_sub_ = n.subscribe<nav_msgs::Odometry>("/odom",10,&odom_callback);
    scan_sub_ = n.subscribe<sensor_msgs::LaserScan>("/scan",50, &scan_callback);
    g_odom_pub = n.advertise<nav_msgs::Odometry>("mbot/odometry", 1000);
    icp_map_pub = n.advertise<nav_msgs::OccupancyGrid> ( "mapping/grid_map", 1 );


    //std::thread thread_close_loop(threadCloseLoopFunc);
    std::thread thread_map_pub(threadMapPubFunc);
    std::thread thread_map_build(threadMapBuildFunc);

    ROS_INFO("start!");
    ros::spin();

    pthread_mutex_destroy(&update_map_mutex);
    pthread_mutex_destroy(&map_mutex);
    pthread_mutex_destroy(&get_init_pose_mutex);

    if(!config.SLAM_MODEL_MIM_())return 0;

    cout<<"[write] start write loop data!"<<endl;
    ofstream loop_benchmark_file("/home/ray/loop_output.txt", ios::app);
    ofstream loop_pose_file("/home/ray/pose_loop_output.txt", ios::app);
    for(int n=0;n<saveframe_buf.size();n++) {
        writeBenchMarkFile(loop_benchmark_file,saveframe_buf[n]);
        writePoseFile(loop_pose_file,saveframe_buf[n]);
    }
    cout<<"[write] write loop success!"<<endl;
    cout<<"[save] start save map!"<<endl;
    saveMap(map_image_save_dir, icp_map_image_save_dir, map_config_save_dir);




}