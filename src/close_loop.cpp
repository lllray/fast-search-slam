//
// Created by ray on 19-8-30.
//

#include "close_loop.h"

using namespace std;
using namespace cv;
//vector<Eigen::Vector2d> cloudFilter(const vector<Eigen::Vector2d> &pts_cloud){
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    cloud->points.resize(pts_cloud.size());
//    for(int i=0;i<pts_cloud.size();i++) {
//        cloud->points[i].x = pts_cloud.at(i)[0];
//        cloud->points[i].y = pts_cloud.at(i)[1];
//        cloud->points[i].z = 0;
//    }
//
//    cout<<"Close Loop cloud SIZE="<<cloud->points.size()<<endl;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZ>);//
//    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
//    voxelgrid.setInputCloud (cloud);
//    voxelgrid.setLeafSize (0.05f, 0.05f, 0.00f);
//    voxelgrid.filter (*cloud_after_voxelgrid);
//
//    vector<Eigen::Vector2d> output_cloud;
//    output_cloud.reserve(cloud_after_voxelgrid->points.size());
//    output_cloud.clear();
//    cout<<"Close Loop cloud_after_voxelgrid SIZE="<<cloud_after_voxelgrid->points.size()<<endl;
//    for (size_t i = 0; i < cloud_after_voxelgrid->points.size(); ++i)
//    {
//        Eigen::Vector2d pt(cloud_after_voxelgrid->points[i].x,cloud_after_voxelgrid->points[i].y);
//        output_cloud.push_back(pt);
//    }
//    return output_cloud;
//}


//vector<Eigen::Vector2d> closeLoopCheck(SubMap &submap){
//
//    vector<Eigen::Vector2d> cloud=cloudFilter(submap.cloud_buf);
//    return cloud;
//
//}

int get_loop_id(int cur_id,Eigen::Matrix2d &R, Eigen::Vector2d &t){
    int best_id=-1;
    double min_dist=config.CLOSE_LOOP_FIND_LOOP_MAX_DISTANCE_();
    if(frame_buf.size()>=config.CLOSE_LOOP_FIND_LOOP_MAX_ID_DIFF_()) {
        for (int i = cur_id-config.CLOSE_LOOP_FIND_LOOP_MAX_ID_DIFF_(); i >= 0; i--) {
            //ROS_INFO("[loop] frame_buf[f_frame_id].pose.t is (%f,%f)",frame_buf[f_frame_id].pose.t[0],frame_buf[f_frame_id].pose.t[1]);
            double dist = (frame_buf[i].pose.t - t).norm();
            if (dist < 0.1) return i;
            else if (dist < min_dist) {
                best_id = i;
                min_dist = dist;
            }
        }
    }else{
        ROS_ERROR("[loop] frame_buf size error is %d",frame_buf.size());
    }

    return best_id;
}
MPose loop_pose;
Mat IcpMapTemp;
void update_map(int loop_id,int cur_id,G2OSolver &solver){
    loop_pose.id=cur_id;
    //update cur_pose
    loop_pose.t=solver.GetCorrections()[cur_id-loop_id].t;
    loop_pose.theta=solver.GetCorrections()[cur_id-loop_id].theta;
    loop_pose.R=thetaToR(loop_pose.theta);
    ROS_INFO("[update] wait start update loop pose");
    if(config.CLOSE_LOOP_DEBUG_()){
        update_map_state=2;
        update_map_id=cur_id;
    }else{
        //开始更新最近位姿
        update_map_state=1;
        ros::Rate r(100);
        while(update_map_state!=2) {
            ROS_INFO("[update] update_map_state=%d  update_map_id=%d", update_map_state, update_map_id);
            ros::spinOnce();
            r.sleep();
        }
    }

    //最近姿态更新完成
    if(update_map_state==2){

    ROS_INFO("[update]  start update all pose size is:%d",update_map_id-loop_id);

        for(int i=update_map_id;i>loop_id;i--){
           // ROS_INFO("[update]   update id is:%d",i);
            if(i>cur_id){
                frame_buf[i].pose.t=loop_pose.t+ frame_buf[i].pose.t-frame_buf[loop_pose.id].pose.t;
                frame_buf[i].pose.R=frame_buf[loop_pose.id].pose.R.inverse()*frame_buf[i].pose.R*loop_pose.R;
                frame_buf[i].pose.theta=rToTheta(frame_buf[i].pose.R);

                saveframe_buf[i].pose=frame_buf[i].pose;
            }else{
                frame_buf[i].pose.t=solver.GetCorrections()[i-loop_id].t;
                frame_buf[i].pose.theta=solver.GetCorrections()[i-loop_id].theta;
                frame_buf[i].pose.R=thetaToR(frame_buf[i].pose.theta);

                //闭环之间的info_k设定为best　不再进行回环
                frame_buf[i].info_k=0.01;

                saveframe_buf[i].pose=frame_buf[i].pose;
            }
        }
        Mat IcpMapZero(Size(config.ICPMAP_WIGTH_(), config.ICPMAP_HIGH_()), CV_8UC1, Scalar(0)); //x*y
        //IcpMap=IcpMapZero;
        //IcpMapTemp=IcpMapZero.clone();
        ROS_INFO("[update]  update_map_id is:%d",update_map_id);
        updateMap(true,IcpMapZero, 0, update_map_id);

        //replenish new fame
        if(frame_buf.size()-1>update_map_id){
            ROS_INFO("[uodate] replenish new fame num is %d",frame_buf.size()-1-update_map_id);
            updateMap(true,IcpMapZero, update_map_id+1, frame_buf.size()-1);

        }
        ROS_INFO("[update] update_map start");
        getIcpMapFreeing()=IcpMapZero.clone();
        changeIcpMap();

        if(!config.CLOSE_LOOP_DEBUG_())update_map_state=0;

        ROS_INFO("[update] update_map finish");
    }else{
        ROS_INFO("[update] update_map_state=%d  update_map_id=%d",update_map_state,update_map_id);
    }

}

bool optimize(int loop_id,int cur_id, Eigen::Vector2d &t,double theta){

    G2OSolver solver;

    for(int i=loop_id;i<=cur_id;i++) {

        // add node
        if (i != cur_id) {
            MNode pose(i - loop_id, frame_buf[i].pose.t, frame_buf[i].pose.theta);
            solver.AddNode(&pose);
        } else {
            MNode pose(i - loop_id, t, theta);
            solver.AddNode(&pose);
        }

        //add edge
        if (i > loop_id) {
            Eigen::Vector2d transNoise(config.CLOSE_LOOP_TRANSNOISE_MAX_(), config.CLOSE_LOOP_TRANSNOISE_MAX_());
            double rotNoise = config.CLOSE_LOOP_ROTNOISE_MAX_();
            Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
            covariance.fill(0.);
            covariance(0, 0) = transNoise[0] * transNoise[0];
            covariance(1, 1) = transNoise[1] * transNoise[1];
            covariance(2, 2) = rotNoise * rotNoise;
            covariance*=frame_buf[i].info_k;
            if(config.DEBUG_INFO_())ROS_INFO("[loop] id=%d match info_k is %f",i,frame_buf[i].info_k);
            Eigen::Vector3d from_pose(frame_buf[i - 1].pose.t[0], frame_buf[i - 1].pose.t[1],
                                      frame_buf[i - 1].pose.theta);
            Eigen::Vector3d to_pose(frame_buf[i].pose.t[0], frame_buf[i].pose.t[1], frame_buf[i].pose.theta);
//            Eigen::Vector2d t_diff=frame_buf[i].pose.t-frame_buf[i-1].pose.t;
//            double theta_diff=frame_buf[i].pose.theta-frame_buf[i-1].pose.theta;
//            MEdge edge(i-loop_id-1,i-loop_id,t_diff,theta_diff,covariance);
            MEdge edge(i - loop_id - 1, i - loop_id, from_pose, to_pose, covariance);
            solver.AddConstraint(&edge);
        }
    }

    solver.Compute(cur_id-loop_id);
    ROS_INFO("[g2o optimization success]");


    if(solver.GetCorrections().size()>0) {
        if (config.CLOSE_LOOP_DEBUG_()) {
            //if (config.DEBUG_IMAGE_()||config.CLOSE_LOOP_DEBUG_()||(cur_id>5000)) {
            vector<Eigen::Vector2d> cloud_before_g2o;
            vector<Eigen::Vector2d> cloud_after_g2o;
            ofstream loopfile("/home/ray/debug_loop_output.txt", ios::app);

            for (int i = loop_id; i <= cur_id; i++) {
                cloud_before_g2o.emplace_back(frame_buf[i].pose.t);
                cloud_after_g2o.emplace_back(solver.GetCorrections()[i - loop_id].t);
                //if (config.CLOSE_LOOP_DEBUG_()||(cur_id>5000)) {
                    if (config.CLOSE_LOOP_DEBUG_()) {
                    if (loopfile.fail()) {
                        cout << "fail" << endl;
                    } else {
                        loopfile << setprecision(9) << ros::Time::now() << " "
                                 <<  frame_buf[i].pose.t[0]<< " "
                                 << frame_buf[i].pose.t[1]<< " "
                                 << frame_buf[i].pose.theta<< " "
                                 << solver.GetCorrections()[i - loop_id].t[0]<< " "
                                 <<  solver.GetCorrections()[i - loop_id].t[1]<< " "
                                 <<  solver.GetCorrections()[i - loop_id].theta
                                 << endl;
                    }
                    ROS_INFO("[loop] frame id %d pose before g2o is (%f,%f,%f)", frame_buf[i].pose.id,
                             frame_buf[i].pose.t[0], frame_buf[i].pose.t[1], frame_buf[i].pose.theta);
                    ROS_INFO("[loop] node id %d pose after g2o is (%f,%f,%f)", solver.GetCorrections()[i - loop_id].id,
                             solver.GetCorrections()[i - loop_id].t[0], solver.GetCorrections()[i - loop_id].t[1],
                             solver.GetCorrections()[i - loop_id].theta);
                }
            }
            if(config.DEBUG_IMAGE_()) {
            Mat color_dst=twoCloudToMat(cloud_before_g2o,cloud_after_g2o,config.SHOW_IMAGE_MAX_SQUARE_(),config.SHOW_IMAGE_RESOLUTION_());
            imwrite("/home/ray/my_map/color_map.png", color_dst);
            color_dst.release();
            }
        }

        ROS_INFO("[loop] loop optimization is success");
        clock_t start,finish;
        double totaltime;
        start=clock();
        update_map(loop_id,cur_id,solver);
        finish=clock();
        totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
        std::cout<<"\n|********"<<cur_id<<"帧更新地图的运行时间为"<<totaltime<<"秒！"<<std::endl;
        //cvWaitKey(0);
        return true;
    }
    else return false;
}