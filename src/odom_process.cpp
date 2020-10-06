//
// Created by ray on 19-10-24.
//
#include "odom_process.h"
vector<RobotOdom> robot_odom_buf;
double last_time=0;
RobotOdom last_odom;

void update_last_time(double time){
    last_time=time;
}
void update_last_odom(RobotOdom &odom){
    last_odom=odom;
}
bool get_cur_pose_from_odom(RobotOdom &odom,double cur_time){
    int cur_id=robot_odom_buf.size()-1;
    odom.time=cur_time;
    if(robot_odom_buf[cur_id].time<cur_time) {
        if(config.DEBUG_INFO_())ROS_ERROR("[get cur pose odom] the last odom time < current time");
        return false;
    }
    for(int i=cur_id-1;i>=0;i--){
        if(robot_odom_buf[i].time<cur_time){
            if(robot_odom_buf[i+1].time==cur_time){
                odom=robot_odom_buf[i];
                odom.use=true;
                return true;
            }
            double per_time=robot_odom_buf[i+1].time-robot_odom_buf[i].time;
            if(per_time>1.0){
                if(config.DEBUG_INFO_())ROS_INFO("[get cur pose odom]  long time");
                return false;
            }else if(per_time<=0.0){
                if(config.DEBUG_INFO_())ROS_INFO("[get cur pose odom]  time error 1");
                return false;
            }

            //RobotOdom cur_odom;
            //cur_odom.time=cur_time;
            double k=(cur_time-robot_odom_buf[i].time)/per_time;
            odom.pose[0]=robot_odom_buf[i].pose[0]+k*(robot_odom_buf[i+1].pose[0]-robot_odom_buf[i].pose[0]);
            odom.pose[1]=robot_odom_buf[i].pose[1]+k*(robot_odom_buf[i+1].pose[1]-robot_odom_buf[i].pose[1]);
            double theta=rToTheta(thetaToR(robot_odom_buf[i].pose[2]).inverse()*thetaToR(robot_odom_buf[i+1].pose[2]));
            if(config.DEBUG_INFO_())ROS_INFO("1=%f 2=%f theta=%f",robot_odom_buf[i].pose[2],robot_odom_buf[i+1].pose[2],theta);
            odom.pose[2]=rToTheta(thetaToR(theta*k)*thetaToR(robot_odom_buf[i].pose[2]));
            if(config.DEBUG_INFO_())ROS_INFO("[get cur pose odom] time=%f cur odom x=%f y=%f theta=%f",odom.time,odom.pose[0],odom.pose[1],odom.pose[2]);

            // ROS_INFO("[get init pose] success!");
            odom.use=true;
            return true;
            // odom.pose[2]=rToTheta(thetaToR(robot_odom_buf[i].pose[2]).inverse()*thetaToR(robot_odom_buf[i+1].pose[2]));
        }else if(robot_odom_buf[i].time==cur_time){
            odom=robot_odom_buf[i];
            odom.use=true;
            return true;
        }

    }
    if(config.DEBUG_INFO_())ROS_INFO("[get cur pose odom] no odom before cur_time");
    return false;
}
bool get_init_pose_from_odom(Eigen::Matrix2d &R,Eigen::Vector2d &t,RobotOdom &odom,Eigen::Matrix2d map_R,Eigen::Vector2d map_t){
    if(!last_odom.use){
        update_last_odom(odom);
        if(config.DEBUG_INFO_())ROS_INFO("init last odom!");
        return false;
    }
    else{

        if(odom.time<=last_odom.time) {
            if(config.DEBUG_INFO_())ROS_INFO("[get init pose]  cur time<last time");
            return false;
        }
        else if((odom.time-last_odom.time)>1.0) {
            if(config.DEBUG_INFO_())ROS_INFO("[get init pose]  long time");
            return false;
        }
        else{
            R=thetaToR(last_odom.pose[2]).inverse()*thetaToR(odom.pose[2]);
            double theta=rToTheta(R);
            t[0]=(odom.pose-last_odom.pose)[0];
            t[1]=(odom.pose-last_odom.pose)[1];
            //t=thetaToR(PI / 2)*init_R*thetaToR(last_odom.pose[2]).inverse()*t;
            t=thetaToR(PI / 2)*map_R*thetaToR(last_odom.pose[2]).inverse()*t;
            //double theta=(cur_odom.pose-last_odom.pose)[2];

            if(config.DEBUG_INFO_())ROS_INFO("[get init pose] pose x=%f y=%f theta=%f",t[0],t[1],theta);

            if(abs(t[0])>config.MATCH_DIST_MAX_RANGE_()*4||abs(t[1])>config.MATCH_DIST_MAX_RANGE_()*4||abs(theta)>config.MATCH_ANGEL_MAX_RANGE_()*4) {
                if(config.DEBUG_INFO_())ROS_INFO("[get init pose] pose is out of range");
                return false;
            }
            if(config.DEBUG_INFO_())ROS_INFO("[get init pose] success!");

            return true;
        }

    }
}

void getInitPoseFromOdom(MFrame &frame,Eigen::Matrix2d &R,Eigen::Vector2d &t,Eigen::Matrix2d map_R,Eigen::Vector2d map_t){

    RobotOdom odom;
    clock_t wait_odom_start;
    wait_odom_start = clock();
    //等待odom数据
    while(((robot_odom_buf.size()>1)&&(robot_odom_buf.back().time<frame.time))&&(((double)(clock() - wait_odom_start) / CLOCKS_PER_SEC)<0.1)){
        waitKey(1);
    }
    //如果有可以odom数据
    if((robot_odom_buf.size()>1)&&(robot_odom_buf.back().time>=frame.time)) {

        pthread_mutex_lock(&get_init_pose_mutex);

        if(!get_cur_pose_from_odom(odom,frame.time)){
            if(config.DEBUG_INFO_())ROS_INFO("get new odom fail!");
        }else {
            if(config.DEBUG_INFO_())ROS_INFO("get new odom success!");
            if(get_init_pose_from_odom(R,t,odom,map_R,map_t)){
                config.SET_MATCH_RANGE_K_(1);
                config.SET_MATCH_STEP_K_(1);
            }else{
                ROS_INFO("get init pose from odom fail!");
                R = Eigen::Matrix2d::Identity();
                t[0]=0.0;
                t[1]=0.0;
                config.SET_MATCH_RANGE_K_(4);
                config.SET_MATCH_STEP_K_(2);
            }

        }

        pthread_mutex_unlock(&get_init_pose_mutex);
    }else{

    }
    frame.odom=odom;
    update_last_odom(odom);
}