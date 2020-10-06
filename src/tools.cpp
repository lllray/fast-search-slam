//
// Created by ray on 19-9-17.
//
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件
#include "tools.h"
using namespace std;
using namespace cv;

/**
 * cloud convert to mat, then show the mat for debug
 * @name oneCloudToMat
 * @author ray
 * @param cloud: one of input cloud
 * @param mat_size: mat size is mat_size*mat_size
 * @param resolution: one pixel means real distance
 * @return mat
 * */
Mat oneCloudToMat(vector<Eigen::Vector2d> cloud,int mat_size,double resolution){
    int center=mat_size/2;
    Mat dst(cv::Size(mat_size, mat_size), CV_8UC1, cv::Scalar(0));
    Mat color_dst;
    dst.copyTo(color_dst);
    cvtColor(color_dst, color_dst, CV_GRAY2BGR);
    unsigned char * pPixel = 0;
    for (int i = 0; i < cloud.size();i++) {
        double x, y;
        int r_p_x, r_p_y;
        x = cloud[i][0];
        y = cloud[i][1];
        r_p_x = (int) (x * 100 / resolution) + center;
        r_p_y = (int) (y * 100 / resolution) + center;
        if (r_p_x >= 0 && r_p_x < mat_size && r_p_y >= 0 && r_p_y < mat_size) {
            pPixel = (unsigned char *) color_dst.data + (r_p_y * color_dst.cols + r_p_x) * 3 + 2;
            *pPixel = 255;
        } else {
            //cout<<"x: "<<x<<"  y: "<<y<<endl;
        }
    }
    imshow("color_dst", color_dst);
    cvWaitKey(1);
    dst.release();
    //color_dst.release();
    return  color_dst;
}

/**
 * two cloud convert to mat, then show the mat for debug
 * @name twoCloudToMat
 * @author ray
 * @param cloud1: one of input cloud
 * @param cloud2: one of input cloud
 * @param mat_size: mat size is mat_size*mat_size
 * @param resolution: one pixel means real distance
 * @return mat
 * */
Mat twoCloudToMat(vector<Eigen::Vector2d> cloud1,vector<Eigen::Vector2d> cloud2,int mat_size,double resolution){
    int center=mat_size/2;
    Mat dst(cv::Size(mat_size, mat_size), CV_8UC1, cv::Scalar(0));
    Mat color_dst;
    dst.copyTo(color_dst);
    cvtColor(color_dst, color_dst, CV_GRAY2BGR);
    unsigned char * pPixel = 0;
    for (int i = 0; i < cloud1.size();i++) {
        double x, y;
        int r_p_x, r_p_y;
        x = cloud1[i][0];
        y = cloud1[i][1];
        r_p_x = (int) (x * 100 / resolution) + center;
        r_p_y = (int) (y * 100 / resolution) + center;
        if (r_p_x >= 0 && r_p_x < mat_size && r_p_y >= 0 && r_p_y < mat_size) {
            pPixel = (unsigned char *) color_dst.data + (r_p_y * color_dst.cols + r_p_x) * 3 + 2;
            *pPixel = 255;
        } else {
            //cout<<"x: "<<x<<"  y: "<<y<<endl;
        }
    }
    for (int i = 0; i < cloud2.size();i++) {
        double x, y;
        int r_p_x, r_p_y;
        x = cloud2[i][0];
        y = cloud2[i][1];
        r_p_x = (int) (x * 100 / resolution) + center;
        r_p_y = (int) (y * 100 / resolution) + center;
        if (r_p_x >= 0 && r_p_x < mat_size && r_p_y >= 0 && r_p_y < mat_size) {
            pPixel = (unsigned char *) color_dst.data + (r_p_y * color_dst.cols + r_p_x) * 3 + 1;
            *pPixel = 255;
        } else {
            //cout<<"x: "<<x<<"  y: "<<y<<endl;
        }
    }
    imshow("color_dst", color_dst);
    cvWaitKey(1);
    dst.release();
    //color_dst.release();
    return  color_dst;
}
/**
 * Matrix R convert to theta
 * @name rToTheta
 * @author ray
 * @param R: 2d Matrix
 * @return theta
 * */
double rToTheta(const Eigen::Matrix2d& R){
    Eigen::Matrix3d temp_R;
    temp_R << R(0,0), R(0,1),0,
            R(1,0), R(1,1),0,
            0,0,0;
    Eigen::Vector3d rpy;
    rpy = temp_R.eulerAngles(0, 1, 2);
    return (double)rpy[2];
}
/**
 * theta convert to Matrix R
 * @name thetaToR
 * @author ray
 * @param theta
 * @return 2d Matrix
 * */
Eigen::Matrix2d thetaToR(double theta) {
    Eigen::AngleAxisd rotation_vector(theta, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d R_3 = rotation_vector.toRotationMatrix();
    Eigen::Matrix2d R;
    R << R_3(0, 0), R_3(0, 1),
            R_3(1, 0), R_3(1, 1);
    return R;
}


/**
 * input cloud filter with mode
 * @name cloudFilter
 * @author ray
 * @param pts_cloud
 * @param mode
 * @param param1
 * @param param
 * @return 2d Matrix
 * */
vector<Eigen::Vector2d> cloudFilter(const vector<Eigen::Vector2d> &pts_cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(pts_cloud.size());
    for(int i=0;i<pts_cloud.size();i++) {
        cloud->points[i].x = pts_cloud.at(i)[0];
        cloud->points[i].y = pts_cloud.at(i)[1];
        cloud->points[i].z = 0;
    }

//    cout<<"Close Loop cloud SIZE="<<cloud->points.size()<<endl;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_voxelgrid(new pcl::PointCloud<pcl::PointXYZ>);//
//    pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
//    voxelgrid.setInputCloud (cloud);
//    voxelgrid.setLeafSize (0.05f, 0.05f, 0.00f);
//    voxelgrid.filter (*cloud_after_voxelgrid);

    /*方法五：半径滤波器*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器

    radiusoutlier.setInputCloud(cloud);    //设置输入点云
    radiusoutlier.setRadiusSearch(0.05f);     //设置半径为100的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(10); //设置查询点的邻域点集数小于2的删除

    radiusoutlier.filter(*cloud_after_Radius);
    std::cout << "半径滤波后点云数据点数：" << cloud->points.size() << std::endl;

    vector<Eigen::Vector2d> output_cloud;
    output_cloud.reserve(cloud_after_Radius->points.size());
    output_cloud.clear();
    cout<<"Close Loop cloud_after_voxelgrid SIZE="<<cloud_after_Radius->points.size()<<endl;
    for (size_t i = 0; i < cloud_after_Radius->points.size(); ++i)
    {
        Eigen::Vector2d pt(cloud_after_Radius->points[i].x,cloud_after_Radius->points[i].y);
        output_cloud.push_back(pt);
    }
    return output_cloud;
}

/**
 * write the pose into file ,save in the local cp
 * @name writePoseFile
 * @author ray
 * @param file
 * @param frame
 * @return void
 * */
void writePoseFile(ofstream &file,MSaveFrame frame) {
    if (file.fail()) {
        cout << "[writePoseFile] fail!" << endl;
    } else {
        file << setprecision(9) << frame.ros_time << " " << frame.pose.t[1] - config.INIT_Y_OFFSET_() << " "
             << -(frame.pose.t[0] - config.INIT_X_OFFSET_()) << " " << frame.pose.theta << endl;
    }
}

/**
 * write the BenchMark result into file ,save in the local cp
 * @name writeBenchMarkFile
 * @author ray
 * @param file
 * @param frame
 * @return void
 * */
void writeBenchMarkFile(ofstream &file,MSaveFrame frame) {
    if (file.fail()) {
        cout << "[writeBenchMarkFile] fail" << endl;
    } else {
        file << setprecision(8) << "FLASER" << " " << config.SCAN_SIZE_();
        for (int i = 0; i < frame.scan.ranges.size(); i++) {
            file << setprecision(6) << " " << frame.scan.ranges[i];
        }
        file << setprecision(8) << " " << frame.pose.t[1] - config.INIT_Y_OFFSET_() << " "
                << -(frame.pose.t[0] - config.INIT_X_OFFSET_()) <<
                " " << frame.pose.theta << " " << frame.pose.t[1] - config.INIT_Y_OFFSET_() << " "
                << -(frame.pose.t[0] - config.INIT_X_OFFSET_()) << " " << frame.pose.theta
                << " " << setprecision(9) << frame.ros_time
                << " " << "nohost" << " " << frame.run_time << endl;

        file << setprecision(8) << "ODOM" << " " << frame.pose.t[1] - config.INIT_Y_OFFSET_() << " "
                << -(frame.pose.t[0] - config.INIT_X_OFFSET_()) <<
                " " << frame.pose.theta << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << setprecision(9) << frame.ros_time
                << " " << "nohost" << " " << frame.run_time << endl;

    }
}
