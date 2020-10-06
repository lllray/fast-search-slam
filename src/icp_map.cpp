//
// Created by ray on 19-8-21.
//

#include "icp_map.h"

 using namespace cv;
 using namespace std;
//namespace LIDAR {

vector<SubMap> submap_buf;
vector<MFrame> frame_buf;
vector<MSaveFrame> saveframe_buf;

Mat IcpMap_0;
Mat IcpMap_1;
Mat localIcpUpdateMap;
Mat gloabIcpUpdataMap;

vector<BuildMapMessage> buildMapMessage;
int IcpMapUseID;
Mat& getIcpMapUseing(){
if(IcpMapUseID==0)return IcpMap_0;
else return IcpMap_1;
};
Mat& getIcpMapFreeing(){
    if(IcpMapUseID==0)return IcpMap_1;
    else return IcpMap_0;
};

void changeIcpMap(){
    if(IcpMapUseID==0)IcpMapUseID=1;
    else IcpMapUseID = 0;
}
Point2i world_to_map(Eigen::Vector2d p_w, double resolution) {
    Point2i p_m;
    int k_w2m = (int) (1.0 / resolution);
    if (k_w2m < 1)ROS_INFO("error in world_to_map");
    //else ROS_INFO("k_w2m is %d",k_w2m);
    p_m.x = (int) (p_w[0] * k_w2m + config.ICPMAP_WIGTH_() / 2);
    p_m.y = (int) (p_w[1] * k_w2m + config.ICPMAP_HIGH_() / 2);
    return p_m;
}

struct IcpPoint{
    int x;
    int y;
    uchar value;
    IcpPoint(int xx,int yy, int v):x(xx),y(yy),value(v){}
};
vector<vector<IcpPoint>> icpBlockV;
//Mat icpBlockMap;


    void initIcpBlockMap(int tier,int valve_max);

    void initMap(){
        //初始化创建　全局地图
        IcpMapUseID=0;
        Mat IcpMapZero_0(Size(config.ICPMAP_WIGTH_(), config.ICPMAP_HIGH_()), CV_8UC1, Scalar(0)); //x*y
        IcpMap_0=IcpMapZero_0;
        Mat IcpMapZero_1(Size(config.ICPMAP_WIGTH_(), config.ICPMAP_HIGH_()), CV_8UC1, Scalar(0)); //x*y
        IcpMap_1=IcpMapZero_1;
        Mat localIcpMapZero(Size(config.ICPMAP_WIGTH_(), config.ICPMAP_HIGH_()), CV_8UC1, Scalar(0)); //x*y
        localIcpUpdateMap=localIcpMapZero;
        Mat gloabIcpMapZero(Size(config.ICPMAP_WIGTH_(), config.ICPMAP_HIGH_()), CV_8UC1, Scalar(0)); //x*y
        gloabIcpUpdataMap=gloabIcpMapZero;

        initIcpBlockMap(config.ICPMAP_POINT_ADD_RANGE_(),config.ICPMAP_VALUE_MAX_());

    }

    void initIcpBlockMap(int tier,int valve_max){
        Mat icpBlockMap(Size(tier+1,tier+1),CV_8UC1, Scalar(0));

        double k=(double)valve_max/pow((double)tier,1);
        for(int i=0;i<icpBlockMap.rows;i++){
            for(int j=0;j<icpBlockMap.cols;j++){

                //clear the same point like j=0;
                if(icpBlockMap.at<uchar>(i,j)!=0)continue;

                Eigen::Vector2d pt(i,j);

                double dist=pt.norm();
                double value=pow((double)dist,1)*k;

                if(dist==0){value=1;}
                else if(dist>tier)continue;

                icpBlockMap.at<uchar>(i,j)=(uchar)value;

                if((int)dist==icpBlockV.size()){
                    //零点，只添加一次
                    if(dist==0){
                        vector<IcpPoint> icp_block_temp;
                        icp_block_temp.emplace_back(IcpPoint(i,j,(uchar)value));
                        icpBlockV.emplace_back(icp_block_temp);
                    }
                    else if(i==0){
                        //增加一种距离类型的容器
                        vector<IcpPoint> icp_block_temp;
                        icp_block_temp.emplace_back(IcpPoint(i,j,(uchar)value));
                        icp_block_temp.emplace_back(IcpPoint(i,-j,(uchar)value));

                        icpBlockMap.at<uchar>(j,i)=(uchar)value;
                        icp_block_temp.emplace_back(IcpPoint(j,i,(uchar)value));
                        icp_block_temp.emplace_back(IcpPoint(-j,i,(uchar)value));
                        icpBlockV.emplace_back(icp_block_temp);
                    }else{
                        ROS_ERROR("[Init icpBlock] icpBlockV size error1 !");
                    }

                }else if(dist>icpBlockV.size()){
                    ROS_ERROR("[Init icpBlock] icpBlockV size error2 !");
                }else{
                    icpBlockV[(int)dist].emplace_back(IcpPoint(i,j,(uchar)value));
                    icpBlockV[(int)dist].emplace_back(IcpPoint(i,-j,(uchar)value));
                    icpBlockV[(int)dist].emplace_back(IcpPoint(-i,j,(uchar)value));
                    icpBlockV[(int)dist].emplace_back(IcpPoint(-i,-j,(uchar)value));
                }
            }
        }
        if(icpBlockV.size()!=(tier+1))
            ROS_ERROR("[Init icpBlock] icpPointBlock error size=%d !",icpBlockV.size());
        else {
//            for (int i = 0; i < icpBlockV.size(); i++) {
//
//                ROS_INFO("%d <= dist < %d----------", i, i + 1);
//                for (int j = 0; j < icpBlockV[i].size(); j++)
//                    ROS_INFO("x=%d y=%d value=%d", icpBlockV[i][j].x, icpBlockV[i][j].y, (int) icpBlockV[i][j].value);
//            }

        }
//        imshow("icpBlock",icpBlockMap);
//        waitKey(0);


    }


unsigned char new_hit(unsigned char old,unsigned char cur) {
//        if (old == 0)
//        return (unsigned char) (config.ICPMAP_VALUE_MAX_() * 0.8 + cur * 0.2);
//    else if (old > cur)return (unsigned char) (old * 0.8 + cur * 0.2);
//    else return (unsigned char) old;
    if(old==0) {
        return (unsigned char) (config.ICPMAP_VALUE_MAX_() * (1-config.ICPMAP_HIT_K_()) + cur * config.ICPMAP_HIT_K_());
    }
    else if(old>cur) {
        double den=config.ICPMAP_HIT_K_()*old/config.ICPMAP_VALUE_MAX_();
        //double den=config.ICPMAP_HIT_K_()*old/(old+cur);
        //if(old<5)ROS_INFO("den=%f, old=%d cur=%d out=%d",den,old,cur,(unsigned char)(old*(1-den)+cur*den));
        return (unsigned char)(old*(1-den)+cur*den);
    }
    else return (unsigned char)old;
}

void updateMap(bool build_mode,Mat& map,const std::vector<Eigen::Vector2d> pts_cloud,Eigen::Matrix2d &R, Eigen::Vector2d &t) {
    if (build_mode) {
        vector<Point2i> clear_buf;
        vector<Point3i> add_point_buf;
        vector<Point2i> init_p;
        for (int i = 0; i < pts_cloud.size(); i++) {
            Eigen::Vector2d pt = R * pts_cloud.at(i) + t;
            Point2i p_m(world_to_map(pt, config.ICPMAP_RESOLUTION_()));
            if (p_m.x < 0 || p_m.y < 0 || p_m.x > config.ICPMAP_WIGTH_() - 1 ||
                p_m.y > config.ICPMAP_HIGH_() - 1)//out size
                continue;
            init_p.emplace_back(p_m);
        }
        for (int n = 0; n <= config.ICPMAP_POINT_ADD_RANGE_(); n++) {
            for (int i = 0; i < init_p.size(); i++) {

                unsigned char *pUpdate = 0;
                for (int j = 0; j < icpBlockV[n].size(); j++) {
                    Point2i p_m_c(init_p[i].x + icpBlockV[n][j].x, init_p[i].y + icpBlockV[n][j].y);

                    if (p_m_c.x < 0 || p_m_c.y < 0 || p_m_c.x > config.ICPMAP_WIGTH_() - 1 ||
                        p_m_c.y > config.ICPMAP_HIGH_() - 1)//out size
                        continue;
                    pUpdate = localIcpUpdateMap.data + (p_m_c.y) * localIcpUpdateMap.cols +
                              (p_m_c.x);

                    if ((*pUpdate) == 0) {
                        unsigned char *pPixel = 0;
                        uchar value_temp = icpBlockV[n][j].value;
                        pPixel = map.data + p_m_c.y * map.cols + p_m_c.x;
                        if (((*pPixel) == 0) || (*pPixel) > value_temp) {
                            *pPixel = new_hit(*pPixel, value_temp);
                            *pUpdate = 1;
                            clear_buf.emplace_back(p_m_c);
                        }
                    }
                }
            }
        }
        unsigned char *pUpdate = 0;
        for (int i = 0; i < clear_buf.size(); i++) {
            pUpdate = localIcpUpdateMap.data + clear_buf[i].y * localIcpUpdateMap.cols + clear_buf[i].x;
            *pUpdate = 0;
        }
    } else {
    }
}

void updateMap(bool build_mode,Mat& map,int start_id,int end_id){
    if(build_mode){
        for (int id = start_id; id <= end_id; id++) {
            vector<Point2i> clear_buf;
            vector<Point3i> add_point_buf;
            vector<Point2i> init_p;
            for(int j=0;j<frame_buf[id].cloud.size();j++) {
                Eigen::Vector2d pt = frame_buf[id].pose.R * frame_buf[id].cloud.at(j) + frame_buf[id].pose.t;
                Point2i p_m(world_to_map(pt, config.ICPMAP_RESOLUTION_()));
                if (p_m.x < 0 || p_m.y < 0 || p_m.x > config.ICPMAP_WIGTH_() - 1 ||
                    p_m.y > config.ICPMAP_HIGH_() - 1)//out size
                    continue;
                init_p.emplace_back(p_m);
            }
            //ROS_INFO("[updateMap] id %d update size is:%d",id,frame_buf[id].cloud.size());
            for (int n = 0; n <= config.ICPMAP_CORE_POINT_RANGE_(); n++) {
                for (int i = 0; i < init_p.size(); i++) {

                    unsigned char *pUpdate = 0;
                    for (int j = 0; j < icpBlockV[n].size(); j++) {
                        Point2i p_m_c(init_p[i].x + icpBlockV[n][j].x, init_p[i].y + icpBlockV[n][j].y);

                        if (p_m_c.x < 0 || p_m_c.y < 0 || p_m_c.x > config.ICPMAP_WIGTH_() - 1 ||
                            p_m_c.y > config.ICPMAP_HIGH_() - 1)//out size
                            continue;
                        pUpdate = gloabIcpUpdataMap.data + (p_m_c.y) * gloabIcpUpdataMap.cols +
                                  (p_m_c.x);

                        if ((*pUpdate) == 0) {
                            unsigned char *pPixel = 0;
                            uchar value_temp = icpBlockV[n][j].value;
                            pPixel = map.data + p_m_c.y * map.cols + p_m_c.x;
                            if (((*pPixel) == 0) || (*pPixel) > value_temp) {
                                *pPixel = new_hit(*pPixel, value_temp);
                                *pUpdate = 1;
                                clear_buf.emplace_back(p_m_c);
                            }
                        }
                    }
                }
            }
            unsigned char *pUpdate = 0;
            for (int i = 0; i < clear_buf.size(); i++) {
                pUpdate = gloabIcpUpdataMap.data + clear_buf[i].y * gloabIcpUpdataMap.cols + clear_buf[i].x;
                *pUpdate = 0;
            }
        }

    }else{
    }
}

void buildIcpMap(const sensor_msgs::LaserScan::ConstPtr& scan,MFrame &frame) {
    if (!config.SLAM_MODEL_MIM_())return;

    if (config.DEBUG_INFO_())ROS_INFO("START build icpmap");
    clock_t start, finish;
    double totaltime;
    start = clock();
    updateMap(true, getIcpMapUseing(), frame.cloud, frame.pose.R, frame.pose.t);

    finish = clock();
    totaltime = (double) (finish - start) / CLOCKS_PER_SEC;
    if (config.DEBUG_INFO_())std::cout << "\n|******更新地图的运行时间为" << totaltime << "秒！" << std::endl;


    static ros::Time ros_start_time = scan->header.stamp;
    ros::Time ros_cur_time = scan->header.stamp;
    int frame_id = frame_buf.size();
    frame.pose.id = frame_id;
    frame_buf.emplace_back(frame);

    //debug dataset
    MSaveFrame saveFrame(frame.pose, ros_cur_time, ros_cur_time.toSec() - ros_start_time.toSec());
    saveFrame.scan = *scan;
    saveframe_buf.emplace_back(saveFrame);
    //

    static double arrive_distance = 0;
    //submap 初始化
    if (submap_buf.size() <= 0) {
        //初始 lock 设置为 flase
        ROS_INFO("[submap] add the %d submap", 0);
        SubMap submap_temp(true, frame);
        submap_buf.emplace_back(submap_temp);
        //submap_buf[0].lock=true;
    } else {
        int cur_submap_id = submap_buf.size() - 1;
        if (frame_id > config.CLOSE_LOOP_FIND_LOOP_MAX_ID_DIFF_())
            if (submap_buf[cur_submap_id].lock && ((frame_id - submap_buf[cur_submap_id].frame.pose.id) >
                                                   config.CLOSE_LOOP_FIND_LOOP_MAX_ID_DIFF_() * 0.5)) {
                ROS_INFO("[submap] add the %d submap", cur_submap_id + 1);
                SubMap submap_temp(false, frame);
                submap_buf.emplace_back(submap_temp);
            }
        //arrive_distance+=(t-submap_buf[cur_submap_id].t_buf[submap_buf[cur_submap_id].sub_id-1]).norm();
    }


    if (config.DEBUG_INFO_())ROS_INFO("FINISH build icpmap");
}

void updateBuildMap(const sensor_msgs::LaserScan::ConstPtr& scan,MFrame &frame){
    if(!config.SLAM_MODEL_MIM_())return;
    BuildMapMessage temp_message(scan,frame);
    pthread_mutex_lock(&update_map_mutex);  //加锁
    buildMapMessage.emplace_back(temp_message);
    pthread_mutex_unlock(&update_map_mutex);  //加锁
}