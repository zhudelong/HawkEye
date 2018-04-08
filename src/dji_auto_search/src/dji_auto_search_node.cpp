#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "dji_auto_search/searchers.h"

using namespace DJI::onboardSDK;
using namespace cv;
std::vector<PotentialArea> path_points;
std::vector<PotentialArea> path_planed;
void light_weight_map(searchers* dji_searcher);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dji_auto_search");
    ros::NodeHandle nh;
    ros::NodeHandlePtr node_;
    node_ = boost::make_shared<ros::NodeHandle>("~");
    ROS_INFO("dji_auto_search_node init successfully!");
    searchers* dji_searcher = new searchers(nh,node_);
    ros::AsyncSpinner spinner(4);
    spinner.start();
#ifdef MAIN_LOOP
    sleep(5);
    dji_searcher->init_map_regions();
    ros::Rate loop_rate(10);
    while(true){
        if(searchers::flag_start_mission && !searchers::flag_abort_landing &&
                !searchers::flag_abort_mission && !searchers::flag_search_task_end){
            /// 1. Prepare----------------------------------------------------------------------------------------------------
            if(!dji_searcher->drone_->activate()){
                char info[] = "activate failed, please check the id and key!\n";
                dji_searcher->show_info(info);
                break;
            }

            while(dji_searcher->drone_->rc_channels.mode < 7000){
                char info[] = "control permission denied, check if F mode if specified!\n";
                dji_searcher->show_info(info);
                usleep(100000); // check F mode 5 times/s
            }

            if(!dji_searcher->drone_->request_sdk_permission_control()){
                char info[] = "request_sdk_permission_control failed, please restart!\n";
                dji_searcher->show_info(info);
                break;
            }
            dji_searcher->drone_->drone_arm();
            usleep(20000);
            dji_searcher->drone_->takeoff();
            sleep(dji_searcher->take_off_time);
            dji_searcher->drone_->gimbal_angle_control(0, -900, 0, 10);
            char take_off_info[] = "take off successfully!\n";

            dji_searcher->show_info(take_off_info);
            for(int i = 0; i < 50 * dji_searcher->go_up_time; i++){
                dji_searcher->drone_->velocity_control(1, 0, 0, dji_searcher->go_up_speed, 0); // go up at 5m/s
                usleep(20000);
            }


            /// 2. start light weight map-----------------------------------------------------------------------------------
            dji_searcher->local_position_navigation(dji_searcher->map_anchor[0], MAP_BOUND, 0, 6);// 6-para
            light_weight_map(dji_searcher);
            if(!path_points.empty()){
                // carry on the first plan
                PotentialArea start;
                start.x = dji_searcher->drone_->local_position.x;
                start.y = dji_searcher->drone_->local_position.y;
                path_points.push_back(start);

                if(dji_searcher->get_optimal_path(path_points, path_planed)){
                    char path_info[80];
                    sprintf(path_info, "path plan success:%d targets are found!", int(path_planed.size()));
                    dji_searcher->show_info(path_info);
                }
            }else{
                // carry on the second plan
                char info[] = "path points is empty: no path points for planning\n";
                dji_searcher->show_info(info);
                /// this branch must be dealed with/////////////////////////////////////
            }


            dji_searcher->tag_sender_switcher = true;
            /// 3. visual servoing to target point--------------------------------------------------------------------------------------
            for( int i = 0; i <int(path_planed.size()); i++){

                /// 3.1 fly to target point----up to 10 and then the next
                int ret_target = dji_searcher->fly_to_target_point(path_planed[i], dji_searcher->fly_height, ERR_BOUND, dji_searcher->fly_to_target_time);
                if( ret_target == -1){
                    continue;
                }else if(ret_target == 0){
                    char info[] = "the drone still not arrived at target after fly_to_target_point!\n";
                    dji_searcher->show_info(info);
                }else{
                    char info[] = "the drone perfectly arrived at target after fly_to_target_point!\n";
                    dji_searcher->show_info(info);
                }


                /// 3.2 start visual servoing to rect center
                float yaw = 0;
                int ret_serv = dji_searcher->visual_servoing_down(path_planed[i], yaw, dji_searcher->halt_height, ERR_BOUND);
                if(ret_serv == -1){
                    continue;
                }else if( ret_serv == 0){
                    char info[] = "there may be a fasle positive since detect count < 3!\n";
                    dji_searcher->show_info(info);
                }

                // keep back off direction to the runway
                if(yaw > 0){
                    yaw -= 180;
                }
                dji_searcher->adjust_angle(yaw);

                // bridge
                if(path_planed[i].area_type == Area::BRIDGE){
                    /// 3.3 check safety
                    if(dji_searcher->check_back_safety(2)){
                        dji_sdk::LocalPosition halt_point = dji_searcher->drone_->local_position;
                        if(!dji_searcher->back_off(halt_point, dji_searcher->back_distance + 2.5, yaw, ERR_BOUND)){
                            continue;
                        }
                        dji_sdk::LocalPosition cur = dji_searcher->drone_->local_position;
                        cur.z = dji_searcher->srh_height;
                        dji_searcher->go_down(cur,yaw,ERR_BOUND);
                        dji_searcher->search_tags(Area::BRIDGE);
                        path_planed[i].is_covered = true;
                    }else{
                        continue;
                    }
                }

                if(path_planed[i].area_type == Area::HOUSE){
                    /// 3.3 check safety
                    if(dji_searcher->check_back_safety(2)){
                        dji_sdk::LocalPosition halt_point = dji_searcher->drone_->local_position;
                        if(!dji_searcher->back_off(halt_point, dji_searcher->back_distance + 3, yaw, ERR_BOUND)){
                            continue;
                        }
                        dji_sdk::LocalPosition cur = dji_searcher->drone_->local_position;
                        cur.z = dji_searcher->srh_height;
                        dji_searcher->go_down(cur,yaw,ERR_BOUND);
                        dji_searcher->search_tags(Area::HOUSE);
                        path_planed[i].is_covered = true;
                    }else{
                        continue;
                    }

                }
            }

            if(!path_points.empty()){
                if(dji_searcher->height_status){
                    dji_searcher->adjust_angle(0);
                    dji_sdk::LocalPosition loc_w = dji_searcher->drone_->local_position;
                    loc_w.z = dji_searcher->fly_height;
                    dji_searcher->local_position_navigation(loc_w,ERR_BOUND,0,3);// go up 10

                    // go to #0 anchor
                    loc_w.x = dji_searcher->map_anchor[0].x-3;
                    loc_w.y = dji_searcher->map_anchor[0].y-5;
                    loc_w.z = dji_searcher->fly_height;
                    dji_searcher->local_position_navigation(loc_w,ERR_BOUND,0,dji_searcher->fly_to_anchor_time);
                    loc_w.z = 3.5;

                    // go down
                    dji_searcher->go_down(loc_w,0,ERR_BOUND);
                    dji_searcher->drone_->gimbal_angle_control(0,-450,0,10);
                    sleep(1);

                    // go right
                    for(int i = 0; i < 50 * dji_searcher->right_fly_time; i++){
                        dji_searcher->drone_->velocity_control(1, 0, 2, 0, 0);
                        usleep(20000);
                    }

                    for(int i = 0; i < 50 * (dji_searcher->right_fly_time - 7); i++ ){
                        dji_searcher->drone_->velocity_control(1, 2, 0, 0, 0);
                        usleep(20000);
                    }
                }
            }

            dji_searcher->tag_sender_switcher = false;


            // fly to 20m and go to region 2
            dji_sdk::LocalPosition back = dji_searcher->drone_->local_position;
            back.z = dji_searcher->map_height;
            dji_searcher->local_position_navigation(back,ERR_BOUND,0,4);
            dji_searcher->local_position_navigation(dji_searcher->map_region[1],ERR_BOUND,0,6);


            /// 4. finish the search task
            char info5[] = "search task ended!";
            dji_searcher->show_info(info5);
            sleep(1);

            char info_wait[] = "start driving right now!";
            dji_searcher->show_info(info_wait);
            sleep(1);
            for(int n =0;n<10;n++){
                dji_dispatcher::TaskState state;
                state.is_finished = true;
                dji_searcher ->task_state_publisher.publish(state);
                usleep(100000);
            }
            sleep(3);
            break;
        }
        loop_rate.sleep();
    }
    sleep(5);
    spinner.stop();
    //ros::shutdown();
#endif

#ifdef WEB_CAMERA
    std::vector<cv::RotatedRect> targets[GRID_WIDTH][GRID_HEIGHT];
    dji_searcher->targets_ptr = &(targets[0][0]);
    dji_searcher->filter_counter = 0;
    dji_searcher->filter_switcher  = true;
    double t = (double)cv::getTickCount();
    for(int i = 1; i <= 6; i++){
        char path[50];
        sprintf(path,"/home/zhudelong/img/bagfile/original_%d.jpg",i);
        cv::Mat ori = imread(path);
        dji_searcher->get_candidate_areas(ori);
        imshow("ori",ori);
    }
    double t2 = ((double)getTickCount() - t)/getTickFrequency();
    std::cout << "time:" <<t2<<std::endl;

    dji_searcher->select_search_target(targets, path_points, cv::Point(10,10));

    if(!path_points.empty()){
        int idx = path_points.size();
        path_planed.push_back(path_points[idx-1]);
    }
    dji_searcher->get_optimal_path(path_points, path_planed);



    for(int i = 0; i < int(path_planed.size()); i++)
    {
        char info[100];
        sprintf(info,"The path is: %f, %f, %f", path_planed[i].x, path_planed[i].y, path_planed[i].rect.angle);
        std::cout << info;
    }
    waitKey(0);




#endif

#ifdef GUIDANCE_TEST
    sleep(2);
    while(true)
    {
        dji_searcher->test_guidance();
        sleep(100000);
    }

#endif

    ros::waitForShutdown();
    //clear
    delete dji_searcher;
    dji_searcher = NULL;
    //node_.reset(NULL);
    return 0;
}

void light_weight_map(searchers *dji_searcher)
{
    for(int i = 0; i < SUB_AREAS_NUM; i++)
    {
        // show information
        char info[100];
        sprintf(info,"Local %d---%f;%f",i, dji_searcher->map_anchor[i].x,dji_searcher->map_anchor[i].y);
        dji_searcher->show_info(info);

        // go to target point
        dji_searcher->local_position_navigation(dji_searcher->map_anchor[i], MAP_BOUND, 0, dji_searcher->fly_to_anchor_time);// 6-para
        sleep(1); // let drone keep stable

        // start collect data
        char start[] = "start capturing pictures!\n";
        dji_searcher->show_info(start);

        // get most possible area for searching
        std::vector<cv::RotatedRect> targets[GRID_WIDTH][GRID_HEIGHT];
        dji_searcher->targets_ptr = &(targets[0][0]);
        dji_searcher->filter_counter = 0;
        dji_searcher->filter_switcher  = true;
        for( int i = 0; i < 20; i++ )
        {
            sleep(2);
            if(dji_searcher->filter_switcher == false && dji_searcher->filter_counter == 0){
                break;
            }
        }
        char end[] = "end capturing pictures!\n";
        dji_searcher->show_info(end);

        if(dji_searcher->test_type != 2){
            dji_searcher->select_search_target(targets, path_points);
        }

        dji_searcher->targets_ptr = NULL;
    }


    if(dji_searcher->test_type == 2){
        for(int i = 0; i < SUB_AREAS_NUM; i++){

            // show information
            char info[100];
            sprintf(info,"Local %d---%f;%f",i, dji_searcher->test_anchor[i].x,dji_searcher->test_anchor[i].y);
            dji_searcher->show_info(info);
            // go to target point
            dji_searcher->local_position_navigation(dji_searcher->test_anchor[i], MAP_BOUND, 0, dji_searcher->fly_to_anchor_time);// 6-para
            sleep(1); // let drone keep stable
            dji_searcher->filter_counter = 0;
            dji_searcher->filter_switcher  = true;
            for( int i = 0; i < 20; i++ ){
                sleep(2);
                if(dji_searcher->filter_switcher == false && dji_searcher->filter_counter == 0){
                    break;
                }
            }
            dji_searcher->search_tags(Area::BRIDGE);
        }
    }
}


#ifdef OFFLINE_TEST_CV
sleep(5);
std::vector<PotentialArea> path_points;
cv::namedWindow("MODULE TEST", cv::WINDOW_AUTOSIZE);
for( int i =0; i < 4; i++)
{
    ROS_INFO("NEXT");
    sleep(3);
    std::vector<cv::RotatedRect> targets[32][24];
    dji_searcher->targets_ptr = &(targets[0][0]);
    dji_searcher->filter_counter = 0;
    dji_searcher->filter_switch  = true;
    sleep(2);
    dji_searcher->filter_switch = false;
    bool sub = false;
    if(i == 0){
        sub = dji_searcher->get_sub_potential_areas(targets, path_points, cv::Point2f(12.5,12.5));
    }else if (i == 1) {
        sub = dji_searcher->get_sub_potential_areas(targets, path_points, cv::Point2f(37.5,12.5));
    }else if(i == 2){
        sub = dji_searcher->get_sub_potential_areas(targets, path_points, cv::Point2f(37.5,37.5));
    }else if(i == 3){
        sub = dji_searcher->get_sub_potential_areas(targets, path_points, cv::Point2f(12.5,37.5));
    }

    // show result
    RNG rng(12345);
    Mat drawing  = Mat::zeros(cv::Size(640,480), CV_8UC3);
    for (size_t i = 0; i < path_points.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        //drawContours(drawing, contours_poly, (int) i, color, 1, 8, vector<Vec4i>(), 0, Point());
        cv::Point2f rect_points[4];
        path_points[i].rect.points(rect_points);
        for( int j = 0; j < 4; j++ )
        {
            line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        }
        ROS_INFO("The cordi is: %f, %f, %f, %f, %f", path_points[i].x, path_points[i].y,
                 path_points[i].rect.size.width, path_points[i].rect.size.height, path_points[i].rect.angle);
    }
    cv::imshow("MODULE TEST",drawing);
}

PotentialArea last_area;
last_area.x = 12.5;
last_area.y = 37.5;
path_points.push_back(last_area);
dji_searcher->get_optimal_path(path_points, path_planed);
for(int i = 0; i < int(path_planed.size()); i++)
{
    ROS_INFO("The path is: %f, %f, %f", path_planed[i].x, path_planed[i].y, path_planed[i].rect.angle);
}
#endif


#ifdef OFFLINE_TEST_CT
sleep(3);
ros::Rate loop_rate(10);
while(true)
{
if(searchers::flag_start_mission && !searchers::flag_abort_landing &&
!searchers::flag_abort_mission && !searchers::flag_search_task_end)
{
    // 1. Prepare
    if(!dji_searcher->drone_->activate())
    {
        ROS_INFO("the drone is still not activated!");
        ROS_INFO("please check the id and key!");
        break;
    }
    if(!dji_searcher->drone_->request_sdk_permission_control())
    {
        ROS_INFO("control permission denied, check if F mode if specified!");
        /// send to mobile
        break;
    }

    dji_searcher->drone_->takeoff();
    sleep(4); // wait for record origin
    std::string info = "chenggongla!";
    dji_searcher->drone_->send_to_mobile(info);

    dji_searcher->drone_->gimbal_angle_control(0, -900, 0, 10);
    sleep(2);


    // 2. Map
    // light_weight_map(dji_searcher);

    // 3. Path plan
    PotentialArea last_area;
    last_area.x = 2.563573; last_area.y =  32.289040;
    last_area.rect.size.width = 168.343796; last_area.rect.size.height = 100.629868;
    last_area.rect.angle = -24.722837;
    path_planed.push_back(last_area);

    PotentialArea last_area2;
    last_area2.x = 13.665184; last_area2.y = 19.070341;
    last_area2.rect.size.width = 139.742538; last_area2.rect.size.height = 138.596634;
    last_area2.rect.angle = -9.122897;
    path_planed.push_back(last_area2);


    PotentialArea last_area3;
    last_area3.x = 13.106299; last_area3.y = 5.379434;
    last_area3.rect.size.width = 63.481880; last_area3.rect.size.height = 130.541183;
    last_area3.rect.angle = -8.812902;
    path_planed.push_back(last_area3);

    PotentialArea last_area4;
    last_area4.x = 30.864927; last_area4.y = 7.280170;
    last_area4.rect.size.width = 186.939804; last_area4.rect.size.height = 137.428192;
    last_area4.rect.angle = -24.381811;
    path_planed.push_back(last_area4);



    //path_points.push_back(last_area);
    //dji_searcher->get_optimal_path(path_points, path_planed);

    // 4. Carry on search task
    for( int i = 0; i <int(path_planed.size()); i++)
    {
        if(!dji_searcher->fly_to_target_point(path_planed[i], FLY_HEIGHT, ERR_BOUND))
        {
            ROS_ERROR("I cannot fly to that area!");
            /// send to mobile
            break;
        }
        int ret = dji_searcher->adjust_oritation(dji_searcher->global_image, FLY_HEIGHT, HAT_HEIGHT, ERR_BOUND);
        if(ret == -1){ path_planed[i].is_covered == true; continue;}
        else if(ret == 0) {
            ROS_ERROR("I cannot fly to that area!");
            /// send to mobile
            path_planed[i].is_covered = true;
            continue;
        }
        // carry on search task
        dji_searcher->search_house();
    }
    dji_dispatcher::TaskState state;
    state.is_finished = true;
    dji_searcher ->task_state_publisher.publish(state);
    break;
}


loop_rate.sleep();
}
// spinner.stop();
//ros::shutdown();
#endif
