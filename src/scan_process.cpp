//
// Created by ray on 19-7-5.
//
#include <scan_process.h>

Eigen::Vector2d scanToWorld(int i,double dist){
    //double x,y;
    //if(config.DATASET_TYPE_())
    switch (config.DATASET_TYPE_()){
        case 1:
        case 2: {
            double theta, rho;
            Eigen::Vector2d output;
            theta = (i / 2.0 - 45) * PI / 180;
            rho = dist;
            output[0] = rho * sin(theta);
            output[1] = -rho * cos(theta);
            return output;
        }break;
        case 3:
        case 4:
        case 7:{
            double theta, rho;
            Eigen::Vector2d output;
            theta = (i) * PI / 180;
            rho = dist;
            output[1] = rho * sin(theta);
            output[0] = rho * cos(theta);
            return output;
        }break;
        case 5:
        case 6:{
            double theta,rho;
            Eigen::Vector2d output;
            theta = (i/2.0)*PI/180;
            rho = dist;
            output[1] = rho*sin(theta);
            output[0] = rho*cos(theta);
            return output;
        }break;

    }

}


//使用体素滤波
vector<Eigen::Vector2d> scanToCloud(sensor_msgs::LaserScan scan,int voxel_r) {
    if(config.USE_VOXEL_FILTER_()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->points.resize(scan.ranges.size());
        for (int i = 0; i < scan.ranges.size(); i += voxel_r) {
            if ((scan.ranges[i] > 0) && (scan.ranges[i] < config.SCAN_MAX_DISTANCE_()) &&
                (scan.ranges[i] > config.SCAN_MIN_DISTANCE_())) {

                Eigen::Vector2d pt = scanToWorld(i, scan.ranges[i]);
                cloud->points[i].x = pt[0];
                cloud->points[i].y = pt[1];
                cloud->points[i].z = 0;
            }

        }
        if (config.DEBUG_INFO_())cout << "cloud SIZE=" << cloud->points.size() << endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZ>);//
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setInputCloud(cloud);
        voxelgrid.setLeafSize(config.VOXEL_FILTER_SIZE_(), config.VOXEL_FILTER_SIZE_(), 0.00f);
        voxelgrid.filter(*cloud_after_voxelgrid);

        vector<Eigen::Vector2d> pts_cloud;
        pts_cloud.reserve(cloud_after_voxelgrid->points.size());
        pts_cloud.clear();
        if (config.DEBUG_INFO_())cout << "cloud_after_voxelgrid SIZE=" << cloud_after_voxelgrid->points.size() << endl;
        for (size_t i = 0; i < cloud_after_voxelgrid->points.size(); ++i) {
            Eigen::Vector2d pt(cloud_after_voxelgrid->points[i].x, cloud_after_voxelgrid->points[i].y);
            if (pt.norm() < config.SCAN_MIN_DISTANCE_() || pt.norm() > config.SCAN_MAX_DISTANCE_())continue;
            pts_cloud.push_back(pt);
        }
        //ROS_ERROR("!!!");
        return pts_cloud;
    }else{
        vector<Eigen::Vector2d> pts_cloud;
        pts_cloud.reserve(scan.ranges.size());
        pts_cloud.clear();
        for (int i = 0; i < scan.ranges.size(); i += voxel_r) {
            if ((scan.ranges[i] > 0) && (scan.ranges[i] < config.SCAN_MAX_DISTANCE_()) &&
                (scan.ranges[i] > config.SCAN_MIN_DISTANCE_())) {
                Eigen::Vector2d pt = scanToWorld(i, scan.ranges[i]);
                pts_cloud.push_back(pt);
            }

        }
        return pts_cloud;
    }

}

//vector<Eigen::Vector2d> scanToCloud(sensor_msgs::LaserScan scan,int voxel_r) {
//    vector<Eigen::Vector2d> pts_cloud;
//    pts_cloud.reserve(scan.ranges.size());
//    pts_cloud.clear();
//    for(int i=0;i<scan.ranges.size();i+= voxel_r){
//        if ((scan.ranges[i] > 0) && (scan.ranges[i] < config.SCAN_MAX_DISTANCE_())&&(scan.ranges[i]>config.SCAN_MIN_DISTANCE_())){
//
//            pts_cloud.push_back(scanToWorld(i, scan.ranges[i]));
//        }
//
//    }
//    return pts_cloud;
//
//}
