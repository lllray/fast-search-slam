//
// Created by ray on 19-7-5.
//

#include <icp.h>
#include <math.h>

using namespace cv;
using namespace std;
//namespace LIDAR

ICP::ICP()
{

}

//返回当前层新增加点个数
int get_point_nn(int n,int n_iter,int seed,int size){

    if(n_iter==n)return seed;
    return (int)(seed*(int)pow(2,(n_iter-n-1)));
}
//返回当前层步长
int get_step_nn(int point_n,int size){

    return (int)(size/point_n);
}
//返回遍历点的初始位置
int get_first_nn(int n,int n_iter,int seed,int size){
    return (size/seed)/(int)pow(2,n_iter-n)-1;
}


int ICP::icp_match_id=0;
double ICP::reject_k_average=0.0;
double ICP::best_score_average=0.0;

bool ICP::Match (
        MFrame &frame,
        Eigen::Matrix2d& R, Eigen::Vector2d& t,Eigen::Matrix2d& l_R,Eigen::Vector2d& l_t,
        const Mat &icp_map,MatchState state
){



    std::vector<Eigen::Vector2d> pts_cloud_temp;
    pts_cloud_temp.reserve ( frame.cloud.size() );//reserve the vector size
//  pts_cloud_temp.assign(pts_cloud.begin(),pts_cloud.end());

    if(config.DEBUG_INFO_())ROS_INFO("START seek");
    State best_state;

    double reject_num=0;
    double cur_squared_dist = 0.0;
    double last_squared_dist = std::numeric_limits<double>::max();
    double squared_distance_best_score=std::numeric_limits <double>::max ();
    double squared_distance_max=frame.cloud.size()*config.ICPMAP_BAD_POINT_VALUE_();


    //int seed_k=config.MATCH_SEED_NUM_();
    //c0=K/(2^(M-1))
    int seed_k=(frame.cloud.size()/pow(2,config.MATCH_TREE_M_()-1))+1;
    int bnb_iter=0;

        for (int i = seed_k; i < frame.cloud.size(); i = seed_k * (int) pow(2, bnb_iter)) {
            bnb_iter++;
        }
    int virtual_szie=seed_k*(int)pow(2,bnb_iter);
    int last_bnb_iter=0;

    int nn_data[bnb_iter+1][3];
    for(int i=0;i<=bnb_iter;i++){
        int nn=bnb_iter-i;
        nn_data[nn][0]=get_point_nn(nn,bnb_iter,seed_k,virtual_szie);
        nn_data[nn][1]=get_step_nn(nn_data[nn][0],virtual_szie);//nn有问题
        nn_data[nn][2]=get_first_nn(nn,bnb_iter,seed_k,virtual_szie);

       // cout<<nn<<" "<<nn_data[nn][0]<<" "<<nn_data[nn][1]<<" "<<nn_data[nn][2]<<" "<<bnb_iter<<" "<<last_bnb_iter<<endl;
    }
    for (int temp_n = bnb_iter; temp_n >= last_bnb_iter; temp_n--)
        for (size_t i = nn_data[temp_n][2]; i < frame.cloud.size(); i += nn_data[temp_n][1]) {
            pts_cloud_temp.push_back(frame.cloud.at(i));
        }



    if(config.DEBUG_INFO_())ROS_INFO("pts_cloud_temp.size=%d",pts_cloud_temp.size());

    int n_iters=2;
    int search_angle_max,search_dist_max;
    double search_angle_range_max,search_dist_range_max;

    if(state==normal){
        n_iters=config.MATCH_MAX_ITERS_();
        search_angle_max = config.MATCH_ANGEL_MAX_STEP_();//*2
        search_dist_max = config.MATCH_DIST_MAX_STEP_();//*2
        search_angle_range_max = config.MATCH_ANGEL_MAX_RANGE_();
        search_dist_range_max = config.MATCH_DIST_MAX_RANGE_();
    }else if(state==relocation) {
        n_iters=config.RELOCATION_MAX_ITERS_();
        search_angle_max = config.RELOCATION_ANGEL_MAX_STEP_();//*2
        search_dist_max = config.RELOCATION_DIST_MAX_STEP_();//*2
        search_angle_range_max = config.RELOCATION_ANGEL_MAX_RANGE_();
        search_dist_range_max = config.RELOCATION_DIST_MAX_RANGE_();
    }else if(state==close_loop_check){
        n_iters=config.CLOSE_LOOP_MAX_ITERS_();
        search_angle_max = config.CLOSE_LOOP_ANGEL_MAX_STEP_();//*2
        search_dist_max = config.CLOSE_LOOP_DIST_MAX_STEP_();//*2
        search_angle_range_max = config.CLOSE_LOOP_ANGEL_MAX_RANGE_();
        search_dist_range_max = config.CLOSE_LOOP_DIST_MAX_RANGE_();
    }
    double rate[n_iters];

    for ( int n = 0; n < n_iters; n ++ ) {
        priority_queue<Queue, vector<Queue>,cmp> pq;
        std::vector<State> state_map;
        //当前层
        int bnb_n=bnb_iter;
        int num=0;

        double search_angle_step=search_angle_range_max/pow(search_angle_max,n)/search_angle_max;//angle　略微扩大范围
        double search_dist_x_step=search_dist_range_max/pow(search_dist_max,n)/search_dist_max; //一开始从搜索半径0.2m的范围，之后迭代，提高精度
        double search_dist_y_step=search_dist_range_max/pow(search_dist_max,n)/search_dist_max; //一开始从搜索半径0.2m的范围，之后迭代，提高精度

        if(config.DEBUG_INFO_())ROS_INFO("search_angle_max=%f",search_angle_step);
        if(config.DEBUG_INFO_())ROS_INFO("search_dist_x_step=%f",search_dist_x_step);
        double do_n=0;
        int finsh_n=0;
        int best_n=0;


        //剔除离群点

        if(n!=0&&start_match&&(state!=close_loop_check)){
            if(config.DEBUG_INFO_()||state==close_loop_check)ROS_INFO("iter %d reject point num is %d",n,best_state.rp_buf.size());
            int len=best_state.rp_buf.size();
            if(len>0) {
                best_state.score-= ((len* config.ICPMAP_BAD_POINT_VALUE_())/pts_cloud_temp.size());
                reject_num+=len;

                if(state==close_loop_check) {
                    ROS_INFO("cur reject_k=%f  reject_k_average=%f",(double) len / pts_cloud_temp.size(),reject_k_average);
                    //if ((double) len / pts_cloud_temp.size() >reject_k_average*config.CLOSE_LOOP_REJECT_POINT_K_())return false;
                    if ((double) len / pts_cloud_temp.size() > config.MATCH_REJECT_POINT_MAX_K_())return false;
                }
                else{

                    if ((double) len / pts_cloud_temp.size() > config.MATCH_REJECT_POINT_MAX_K_())return false;
                }
                int temp;
                int mp_i, mp_j;
                for (mp_i = 0; mp_i < len - 1; mp_i++) /* 外循环为排序趟数，len个数进行len-1趟 */
                    for (mp_j = 0; mp_j < len - 1 - mp_i; mp_j++) { /* 内循环为每趟比较的次数，第i趟比较len-i次 */
                        if (best_state.rp_buf[mp_j] < best_state.rp_buf[mp_j + 1]) { /* 相邻元素比较，若逆序则交换（升序为左大于右，降序反之） */
                            temp = best_state.rp_buf[mp_j];
                            best_state.rp_buf[mp_j] = best_state.rp_buf[mp_j + 1];
                            best_state.rp_buf[mp_j + 1] = temp;
                        }
                    }
                if(config.DEBUG_INFO_())ROS_INFO("pts_cloud_temp size before deleate reject point %d",pts_cloud_temp.size());
                for (int rp_i = 0; rp_i < best_state.rp_buf.size(); rp_i++) {
                    if(config.DEBUG_INFO_())ROS_INFO("deleate reject point id %d",best_state.rp_buf[rp_i]);
                    pts_cloud_temp.erase(pts_cloud_temp.begin() + best_state.rp_buf[rp_i]);
                }
                if(config.DEBUG_INFO_())ROS_INFO("pts_cloud_temp size after deleate reject point %d",pts_cloud_temp.size());
            }
            //删除离群点后需要重新更新最优值

        }else{
            best_state.rp_buf.clear();
        }

        //测试密度搜索树效率
        double qf_k[bnb_iter+1][2];

        for(int test_bnb=bnb_iter;test_bnb>=0;test_bnb--) {
            if(test_bnb!=bnb_iter)qf_k[test_bnb][0]=qf_k[test_bnb+1][0];
            else qf_k[test_bnb][0]=0;

            for (size_t i = nn_data[test_bnb][2]; i < pts_cloud_temp.size(); i += nn_data[test_bnb][1]) {
                qf_k[test_bnb][0]++;
            }
            qf_k[test_bnb][1]=(double)pts_cloud_temp.size()/qf_k[test_bnb][0];
            if(test_bnb==0)qf_k[test_bnb][1]=1;
            //cout<<test_bnb<<" "<<qf_k[test_bnb][0]<<" "<<qf_k[test_bnb][1]<<endl;
        }

//优先角度计算可减少计算量
        for(int a=-search_angle_max;a<=search_angle_max;a++) {

            double a_value = a * search_angle_step;
            Eigen::AngleAxisd rotation_vector(a_value, Eigen::Vector3d(0, 0, 1));
            Eigen::Matrix3d R_3_a = rotation_vector.toRotationMatrix();
            Eigen::Matrix2d R_a;
            R_a << R_3_a(0, 0), R_3_a(0, 1),
                    R_3_a(1, 0), R_3_a(1, 1);
            Eigen::Matrix2d R_c = R_a * R;
            Eigen::Matrix2d R_m = R_c * l_R;


            for (int x = -search_dist_max; x <= search_dist_max; x++)
                for (int y = -search_dist_max; y <= search_dist_max; y++) {

                    if (n != 0 && a == 0 && x == 0 && y == 0)continue;//中心点在第二次开始不需要执行

                    double x_value = x * search_dist_x_step;
                    double y_value = y * search_dist_y_step;


                    double sum_squared_dist = 0.0;
                    double squared_dist_Estimate = 0.0;
//                    double a_value = a * search_angle_step;
//                    Eigen::AngleAxisd rotation_vector(a_value, Eigen::Vector3d(0, 0, 1));
//                    Eigen::Matrix3d R_3_a = rotation_vector.toRotationMatrix();
//                    Eigen::Matrix2d R_a;
//                    R_a << R_3_a(0, 0), R_3_a(0, 1),
//                            R_3_a(1, 0), R_3_a(1, 1);
//                    Eigen::Matrix2d R_c = R_a * R;
//                    Eigen::Matrix2d R_m = R_c * l_R;
                    Eigen::Vector2d t_c(x_value, y_value);
                    t_c += t;
                    if ((state == close_loop_check) && (t_c.norm() < config.MATCH_DIST_MAX_RANGE_() * 2) &&
                        abs(rToTheta(R_c)) < config.MATCH_ANGEL_MAX_RANGE_() * 2) {
                        //ROS_INFO("[loop match] t_c=(%f,%f) theta=%f",t_c[0],t_c[1],rToTheta(R_c));
                        continue;
                    }
                    int reject_point_n = 0;
                    State current;
                    for (size_t i = nn_data[bnb_n][2]; i < pts_cloud_temp.size(); i += nn_data[bnb_n][1]) {

                        //Eigen::Vector2d pt=R_c * pts_cloud_temp.at (i) + t_c;
                        Eigen::Vector2d pt = R_m * pts_cloud_temp.at(i) + t_c + l_t;

                        Point2i p_m(world_to_map(pt, config.ICPMAP_RESOLUTION_()));
                        if (p_m.x <= 0 || p_m.y <= 0 || p_m.x >= config.ICPMAP_WIGTH_() - 1 ||
                            p_m.y >= config.ICPMAP_HIGH_() - 1)//out size
                        {
                            //if(config.NAVIGATION_MODEL_MIM_())sum_squared_dist+=( config.ICPMAP_BAD_POINT_VALUE_());
                            continue;
                        }

                        unsigned char *pPixel = 0;
                        pPixel = icp_map.data + p_m.y * icp_map.cols + p_m.x;
                        if (*pPixel == 0) {
                            //reject_point_n++;
                            //将离群点记录
                            //if()
                            current.rp_buf.emplace_back(i);
                            //在自己定义的参数中config.ICPMAP_BAD_POINT_VALUE_()　包含了　config.ICPMAP_POINT_ADD_RANGE_()
                            sum_squared_dist += (config.ICPMAP_BAD_POINT_VALUE_());
                        } else {
                            sum_squared_dist += ((*pPixel));
                            //sum_squared_dist+=bilinearInterpolation(pt,p_m,icp_map,*pPixel);
                        }
                    } // for all pts_cloud_temp

                    //若推入数据成功
                    {

                        double last_score = 0;
                        current.bnb_n = bnb_n;
                        current.sum_squared_dist = sum_squared_dist;
                        current.R = R_c;
                        current.t = t_c;
                        current.x = x;
                        current.y = y;
                        current.a = a;
                        current.score = current.sum_squared_dist / pts_cloud_temp.size();
                        if (current.bnb_n == last_bnb_iter) {//如果到了最后一层
                            if (current.score < squared_distance_best_score) {
                                squared_distance_best_score = current.score;
                                best_state = current;
                                continue;
                            }
                        } else {
                            if (current.score > squared_distance_best_score) {
                                continue;
                            }
                        }
                        //squared_dist_Estimate=current.score*qf_k[bnb_n][1];
                        squared_dist_Estimate = current.score;
                        Queue queue = {num, squared_dist_Estimate};
                        pq.push(queue);
                        state_map.push_back(current);
                        num++;
                    }

                }
        }
        // 测试密度搜索树效率
        do_n=pq.size()*qf_k[bnb_iter][0];
        int all=pq.size()*pts_cloud_temp.size();

        int finsh_max=10;

        if(config.DEBUG_INFO_())ROS_INFO("pq.size() is :%d",pq.size());
        while(pq.size()>0) {

            //从队列中返回第一个元素
            Queue current_queue = pq.top();
            //队列中删除第一个元素
            pq.pop();

            int c_n = current_queue.n;
            State current = state_map[c_n];

            if (current.score > squared_distance_best_score)//判断当前均值是否大于最小值
            {
                continue;
            }

            bnb_n = current.bnb_n - 1;

            int advanve=0;

            if(finsh_n<=finsh_max){
            }
            else{
               if(abs(best_state.a-current.a)>2||abs(best_state.x-current.x)>2||abs(best_state.x-current.x)>2){
                   if(current_queue.squared_dist_Estimate>squared_distance_max){
                       // //测试密度搜索树准确度　
                       //advanve=1;
                       continue;
                   }else {
                       Queue queue = {c_n,squared_distance_max+current_queue.squared_dist_Estimate};
                       pq.push(queue);
                       continue;
                   }
               }
            }

//  //测试密度搜索树效率
//            if(advanve==0)
//            do_n+=qf_k[bnb_n][0]-qf_k[bnb_n+1][0];


            double sum_squared_dist = 0.0;
            double squared_dist_Estimate=0.0;

            for (size_t i = nn_data[bnb_n][2]; i < pts_cloud_temp.size(); i += nn_data[bnb_n][1]) {

                //Eigen::Vector2d pt = current.R * pts_cloud_temp.at(i) + current.t;
                Eigen::Vector2d pt=current.R * l_R * pts_cloud_temp.at (i) + current.t + l_t;

                Point2i p_m(world_to_map(pt,config.ICPMAP_RESOLUTION_()));

                if(p_m.x<=0||p_m.y<=0||p_m.x>=config.ICPMAP_WIGTH_()-1||p_m.y>=config.ICPMAP_HIGH_()-1)//out size
                {
                    //if(config.NAVIGATION_MODEL_MIM_())sum_squared_dist+=(config.ICPMAP_BAD_POINT_VALUE_());
                    continue;
                }
                unsigned char *pPixel = 0;
                pPixel = icp_map.data + p_m.y * icp_map.cols + p_m.x;
                if (*pPixel == 0){
                    current.rp_buf.emplace_back(i);
                    sum_squared_dist+=(config.ICPMAP_BAD_POINT_VALUE_());
                }else {
                    sum_squared_dist +=((*pPixel));
                    //sum_squared_dist+=bilinearInterpolation(pt,p_m,icp_map,*pPixel);
                }

            }

            // for all pts_cloud_temp
            //若推入数据成功
            {
                double last_score=current.score;
                current.bnb_n = bnb_n;
                current.sum_squared_dist += sum_squared_dist;
                current.score = current.sum_squared_dist / pts_cloud_temp.size();
                if (current.bnb_n == last_bnb_iter) {//如果到了最后一层

                    if (current.score < squared_distance_best_score) {
                        squared_distance_best_score = current.score;
                        best_state = current;
                        if (advanve == 1) {
                            ROS_ERROR("GET ONE BEST  num=%d bnb_n=%d a=%f x=%f y=%f score=%f", c_n,
                                      current.bnb_n,
                                      current.a, current.x, current.y, current.score);
                            best_n++;
                        } else {
                            best_n = 1;
                            finsh_n = 0;
                        }

                        if (config.DEBUG_INFO_()) {
                            if (n == 0 &&
                                (abs(current.a) == search_angle_max || abs(current.x) == search_dist_max ||
                                 abs(current.y) == search_dist_max))
                                ROS_ERROR("GET ONE BEST  num=%d bnb_n=%d a=%f x=%f y=%f score=%f", c_n,
                                          current.bnb_n,
                                          current.a, current.x, current.y, current.score);
                            else
                                ROS_INFO("GET ONE BEST  num=%d bnb_n=%d a=%f x=%f y=%f score=%f", c_n,
                                         current.bnb_n,
                                         current.a, current.x, current.y, current.score);
                        }
                        continue;
                    }else {
                        if ((best_n > 0) && finsh_n <= finsh_max) {
                            finsh_n++;
                        }
                        continue;
                    }
                }

                //squared_dist_Estimate=current.score*qf_k[bnb_n][1];
                squared_dist_Estimate=current.score;
                current_queue.squared_dist_Estimate = squared_dist_Estimate;
                pq.push(current_queue);
                state_map[c_n] = current;
            }

        }
        //若推入数据失败 暂时不考虑失败
        // step 3. Check if convergenced.


        R= best_state.R;
        t=best_state.t;


        if(state==normal){
            if(best_n>1)g_qf_rate[n][0]+=1;
            if(all>0)rate[n]=(double)do_n/all;
            //g_bnb_rate[n]+=rate[n];
        }
        if (config.DEBUG_INFO_())ROS_INFO("n_iters=%d --- n=%d all=%d do_n=%f  finsh_n=%d  best_n=%d -------*****************----- best_state.score:=%f",n_iters, n,all,do_n,finsh_n,best_n,best_state.score);
    } // for n_iters

    //测试分辨率分支数准确度
//    {
//        int n=2;
//        bool get_bad=false;
//        double search_angle_step = search_angle_range_max / pow(search_angle_max, n) / search_angle_max;//angle　略微扩大范围
//        double search_dist_x_step =
//                search_dist_range_max / pow(search_dist_max, n) / search_dist_max; //一开始从搜索半径0.2m的范围，之后迭代，提高精度
//        double search_dist_y_step =
//                search_dist_range_max / pow(search_dist_max, n) / search_dist_max; //一开始从搜索半径0.2m的范围，之后迭代，提高精度
//
////        ROS_INFO("search_angle_step=%f", search_angle_step);
////        ROS_INFO("search_dist_x_step=%f", search_dist_x_step);
////        ROS_INFO("search_dist_max=%f", pow(search_dist_max, n));
////        ROS_INFO("search_angle_max=%f", pow(search_angle_max, n));
//        for (int x = -pow(search_dist_max, n); x <= pow(search_dist_max, n); x++) {
//            for (int y = -pow(search_dist_max, n); y <= pow(search_dist_max, n); y++) {
//                for (int a = -pow(search_angle_max, n); a <= pow(search_angle_max, n); a++) {
//
//                    if (n != 0 && a == 0 && x == 0 && y == 0)continue;//中心点在第二次开始不需要执行
//
//                    double x_value = x * search_dist_x_step;
//                    double y_value = y * search_dist_y_step;
//                    double a_value = a * search_angle_step;
//
//                    double sum_squared_dist = 0.0;
//                    double squared_dist_Estimate = 0.0;
//                    Eigen::AngleAxisd rotation_vector(a_value, Eigen::Vector3d(0, 0, 1));
//                    Eigen::Matrix3d R_3_a = rotation_vector.toRotationMatrix();
//                    Eigen::Matrix2d R_a;
//                    R_a << R_3_a(0, 0), R_3_a(0, 1),
//                            R_3_a(1, 0), R_3_a(1, 1);
//
//                    Eigen::Vector2d t_c(x_value, y_value);
//
//                    if ((t_c.norm() < search_dist_max * search_dist_x_step) &&
//                        abs(rToTheta(R_a)) < search_angle_max * search_angle_step) {
//                        // ROS_INFO("[loop match] t_c=(%f,%f) theta=%f",t_c[0],t_c[1],rToTheta(R_a));
//                        continue;
//                    }
//                    Eigen::Matrix2d R_c = R_a * R;
//                    t_c += t;
//                    int reject_point_n = 0;
//
//                    for (size_t i = 0; i < frame.cloud.size(); i++) {
//
//                        Eigen::Vector2d pt = R_c * l_R * frame.cloud.at(i) + t_c + l_t;
//
//                        Point2i p_m(world_to_map(pt, config.ICPMAP_RESOLUTION_()));
//                        if (p_m.x <= 0 || p_m.y <= 0 || p_m.x >= config.ICPMAP_WIGTH_() - 1 ||
//                            p_m.y >= config.ICPMAP_HIGH_() - 1)//out size
//                        {
//                            continue;
//                        }
//
//                        unsigned char *pPixel = 0;
//                        pPixel = icp_map.data + p_m.y * icp_map.cols + p_m.x;
//                        if (*pPixel == 0) {
//                            sum_squared_dist += (config.ICPMAP_BAD_POINT_VALUE_());
//                        } else {
//                            sum_squared_dist += ((*pPixel));
//
//                        }
//                    } // for all pts_cloud_temp
//
//                    //若推入数据成功
//                    {
//
//
//                        double current_score = sum_squared_dist /frame.cloud.size();
//
//                        if (current_score < squared_distance_best_score - 0.5) {
//                            g_bnb_rate[n] += 1;
//                            //ROS_INFO("get one bad!");
//                            ROS_INFO("GET ONE BAD  a=%f x=%f y=%f score=%f best=%f",
//                                     a_value, x_value,y_value,current_score,squared_distance_best_score);
//                            get_bad = true;
//                            break;
//                        }
//                    }
//
//                }
//                if(get_bad)break;
//            }
//            if(get_bad)break;
//        }
//    }
    if(config.DEBUG_INFO_()) {
        ROS_INFO("FINISH seek");
        ROS_INFO("/*******************************/");
    };

    //update frame
    if(state==close_loop_check)reject_num=best_state.rp_buf.size();
    double reject_k=reject_num/frame.cloud.size();

    frame.score=best_state.score;
    frame.reject_k=reject_k;
    if(best_score_average>0)
        frame.info_k=best_state.score/best_score_average;
    else frame.info_k=1;

    if(state==normal){
        if(icp_match_id>=100)icp_match_id=100;
       // double reject_k=reject_num/frame.cloud.size();

        if(config.DEBUG_INFO_()) ROS_INFO("best_state.score=%f  best_score_average =%f",best_state.score,best_score_average);
        reject_k_average=reject_k_average*(double)icp_match_id/(icp_match_id+1)+reject_k/(icp_match_id+1);
        best_score_average=best_score_average*(double)icp_match_id/(icp_match_id+1)+best_state.score/(icp_match_id+1);
        icp_match_id++;

        if(config.DEBUG_INFO_()) ROS_INFO("icp_match_id=%d reject_k_average=%f best_score_average=%f ",icp_match_id,reject_k_average,best_score_average);
        return true;
    }else if(state==relocation) {
        return true;
    }else if(state==close_loop_check){
        //reject_k=reject_num/frame.cloud.size();
        ROS_INFO("best_state.score=%f < best_score_average=%f  reject_k=%f < reject_k_average=%f",best_state.score,best_score_average,reject_k,reject_k_average);

        if((best_state.score<best_score_average*config.CLOSE_LOOP_SCORE_K_())&&(reject_k<=reject_k_average*config.CLOSE_LOOP_REJECT_POINT_K_()))
            return true;
        else return false;


        //if(reject_num/pts_cloud.size()>)

    }
} // match