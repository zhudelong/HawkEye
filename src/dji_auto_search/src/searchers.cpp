#include "dji_auto_search/searchers.h"
#include <math.h>       /* fabs */
using namespace cv;
using namespace std;

bool searchers::flag_start_mission = false;
bool searchers::flag_abort_mission = false;
bool searchers::flag_abort_landing = false;
bool searchers::flag_search_task_end = false;
bool searchers::flag_auto_landing_end = false;
bool searchers::flag_detect_all = false;
bool searchers::flag_limitation_ok = false;
bool searchers::flag_go_back = false;
int  searchers::abort_landing_count = 0;
int  searchers::status_code = 0;

searchers::~searchers()
{
    if(this->drone_ != NULL){
        delete this->drone_;
        this->drone_ = NULL;
    }
    if(this->finder != NULL){
        delete this->finder;
        this->finder = NULL;
    }
}

searchers::searchers(ros::NodeHandle& nh, ros::NodeHandlePtr node_ptr):
    filter_switcher(false),filter_counter(0), gpu_enabled(false),
    targets_ptr(NULL),img_num(0),drone_speed(0.00),tag_sender_switcher(false),
    local_navigation_timeout(6),test_type(1),pixel_meter_map_err(0),grid_x_idx(10),grid_y_idx(10)
{
    // the image_transport_hint requires a private like node handle
    node_  =  node_ptr;
    image_ = boost::make_shared< image_transport::ImageTransport >(*node_);

    // subscribe the apriltag info from apriltag node
    this->tag_global_position_subscriber = nh.subscribe("/tag_global_position", 1, &searchers::GlobalPositionCallback, this);
    this->tag_local_position_subscriber  = nh.subscribe("/tag_local_position",1, &searchers::LocalPositionCallback,this);

    // subscribe guidance info from guidance node
    this->guidance_distance_subscriber   = nh.subscribe("/guidance/obstacle_distance",1,&searchers::GuidanceDistanceCallback,this);
    this->guidance_ultrasonic_subscriber = nh.subscribe("/guidance/ultrasonic",1,&searchers::GuidanceUltrasonicCallback,this);
    this->guidance_bumper_subscriber = nh.subscribe("/guidance/detection_result",1,&searchers::GuidanceBumperCallback,this);

    // subscribe the image and camera info from X3 node
    ros::TransportHints ros_transport_hints(ros::TransportHints().tcpNoDelay());
    image_transport::TransportHints image_transport_hint("raw",ros_transport_hints,(*node_),"image_transport");
    this->x3_image_subscriber = (*image_).subscribe("image",1, &searchers::ImageCallback,this,image_transport_hint);
    this->x3_info_subscriber  = (*node_).subscribe("camera_info",1, &searchers::InfoCallback,this);

    // subscribe global dispatcher's order from client node
    this->global_dispatcher = nh.subscribe("/task_dispatcher",10, &searchers::DispatcherCallback, this, ros_transport_hints);
    this->task_state_publisher = nh.advertise<dji_dispatcher::TaskState>("dji_auto_search/auto_search_state",10);
    this->drone_ = new DJIDrone(nh);
    this->finder = new TargetFinder();
    // 	 parameters
    this->init_parameters(node_);
}

void searchers::init_parameters(ros::NodeHandlePtr node_para)
{
    // init search area position  --  use anti-clock wise
    node_para->param("start_point/x", corners[0].latitude,  0.00);
    node_para->param("start_point/y", corners[0].longitude, 0.00);
    node_para->param("cross_point/x", corners[2].latitude,  0.00);
    node_para->param("cross_point/y", corners[2].longitude, 0.00);
    node_para->param("left_point/x",  corners[3].latitude,  0.00);
    node_para->param("left_point/y",  corners[3].longitude, 0.00);
    node_para->param("right_point/x", corners[1].latitude,  0.00);
    node_para->param("right_point/y", corners[1].longitude, 0.00);
    node_para->param("center_bia/x", center_bia_x, 0.00);
    node_para->param("center_bia/y", center_bia_y, 0.00);
    node_para->param("anchor_center_dis",  anchor_center_dis, 7.5);//10
    node_para->param("tar_diff_dis", tar_diff_dis, 5.00);
    node_para->param("grid_x_idx", grid_x_idx, 10);
    node_para->param("grid_y_idx", grid_y_idx, 10);

    // init node parameters
    node_para->param("take_off_time", take_off_time, 4);
    node_para->param("go_up_speed", go_up_speed, 5);
    node_para->param("go_up_time",  go_up_time,  2);
    node_para->param("fly_to_anchor_time", fly_to_anchor_time, 6);
    node_para->param("fly_to_target_time", fly_to_target_time, 4);
    node_para->param("back_time", back_time, 3);
    node_para->param("back_distance", back_distance, 3.00);
    node_para->param("back_threshold", back_threshold, 2.5);
    node_para->param("go_down_time", go_down_time, 4);
    node_para->param("go_down_threshold", go_down_threshold, 2.00);

    // init search parameter
    node_para->param("test_type",  test_type, 1);
    node_para->param("drone_speed",  drone_speed, 2.00);
    node_para->param("area_width",   area_width, 50.00);
    node_para->param("drone_height", drone_height, 1.5);
    node_para->param("safety_range", safety_range, 2.5);
    node_para->param("local_navigation_timeout",  local_navigation_timeout, 6);
    node_para->param("pixel_meter_map_err",  pixel_meter_map_err, 0.00);

    // key parameter
    node_para->param("map_height",  map_height, 20.00);
    node_para->param("fly_height",  fly_height, 10.00);
    node_para->param("halt_height", halt_height, 6.00);
    node_para->param("srh_height",  srh_height, 2.500);

    node_para->param("height_status",  height_status, 1);
    node_para->param("layout_status",  layout_status, 1);
    node_para->param("right_fly_time",  right_fly_time, 1);



    // init ninety scann data structure
    for(int i = 0; i < 5; i++){
        this->point_distance[i] = 0;
    }

    // init search area grid to 10 by 10
    for(int i = 0; i < 10; i++){
        for(int j = 0; j < 10; j++){
            this->grid[i][j] = true;
        }
    }

    // init cuda device
    int num_dev = cv::gpu::getCudaEnabledDeviceCount();
    if(num_dev < 1){
        ROS_ERROR("No cuda deveice!");
        this->gpu_enabled = false;
        return;
    }
    cv::gpu::DeviceInfo dev_info(0);
    if(!dev_info.isCompatible()){
        ROS_ERROR("GPU module isn't built for this GPU!");
        this->gpu_enabled = false;
        return;
    }
    cv::gpu::setDevice(0);
    gpu::CudaMem host_rgb4(20, 10, CV_8UC1, gpu::CudaMem::ALLOC_PAGE_LOCKED);
    host_rgb4.release();
    this->gpu_enabled = true;
}

void searchers::init_map_regions(){
    for(int i = 0; i < 4; i++){
        if(corners[i].latitude == 0 || corners[i].longitude == 0){
            ROS_ERROR("gps read error! please check carefully!");
        }
        this->map_region[i] = this->gps_convert_ned(corners[i]);
        this->map_region[i].z = this->map_height;
        ROS_INFO("anchor:%f;%f",this->map_region[i].x,this->map_region[i].y);
    }
    float x_center = (map_region[0].x + map_region[2].x)/2 + center_bia_x;
    float y_center = (map_region[0].y + map_region[2].y)/2 + center_bia_y;

    this->map_anchor[0].x = x_center - anchor_center_dis;
    this->map_anchor[0].y = y_center - anchor_center_dis;
    this->map_anchor[0].z = this->map_height;
    this->map_anchor[1].x = x_center - anchor_center_dis;
    this->map_anchor[1].y = y_center + anchor_center_dis;
    this->map_anchor[1].z = this->map_height;
    this->map_anchor[2].x = x_center + anchor_center_dis;
    this->map_anchor[2].y = y_center + anchor_center_dis;
    this->map_anchor[2].z = this->map_height;
    this->map_anchor[3].x = x_center + anchor_center_dis;
    this->map_anchor[3].y = y_center - anchor_center_dis;
    this->map_anchor[3].z = this->map_height;

    ///temp usage
    this->test_anchor[0].x = x_center;
    this->test_anchor[0].y = y_center - anchor_center_dis;
    this->test_anchor[0].z = this->map_height;
    this->test_anchor[1].x = x_center - anchor_center_dis;
    this->test_anchor[1].y = y_center;
    this->test_anchor[1].z = this->map_height;
    this->test_anchor[2].x = x_center;
    this->test_anchor[2].y = y_center + anchor_center_dis;
    this->test_anchor[2].z = this->map_height;
    this->test_anchor[3].x = x_center + anchor_center_dis;
    this->test_anchor[3].y = y_center;
    this->test_anchor[3].z = this->map_height;
}


void searchers::GuidanceBumperCallback(const dji_bumper::DetectionResult& obstacle){
    this->obst_x = obstacle.window_pos_rear.x;
    this->obst_y = obstacle.window_pos_rear.y;
    this->obst_z = obstacle.window_pos_rear.z;
    this->obst_status = obstacle.window_sta_rear;
}

void searchers::GuidanceDistanceCallback(const sensor_msgs::LaserScan& obj_dis)
{
    this->obj_distance = obj_dis;
    for(int i = 0; i < 5; i++){
        this->point_distance[i] = obj_dis.ranges[i];
    }
}
void searchers::GuidanceUltrasonicCallback(const sensor_msgs::LaserScan& ultra_dis)
{
    this->ultrasonic  = ultra_dis;
}

void searchers::DispatcherCallback(const dji_dispatcher::TaskDispatcher& orders){
    searchers::flag_start_mission =  orders.flag_start_mission;
    searchers::flag_abort_mission =  orders.flag_abort_mission;
    searchers::flag_abort_landing =  orders.flag_abort_landing;
    searchers::flag_search_task_end = orders.flag_auto_landing_end;
    searchers::flag_auto_landing_end = orders.flag_auto_landing_end;
    searchers::flag_detect_all = orders.flag_detect_all;
    searchers::flag_limitation_ok = orders.flag_limitation_ok;
    searchers::flag_go_back = orders.flag_go_back;
    searchers::abort_landing_count = orders.abort_landing_count;
    searchers::status_code = orders.status_code;
}

void searchers::GlobalPositionCallback(const std_msgs::String &position)
{
    if(this->tag_sender_switcher){
        std::string info = position.data;
        this->drone_->send_to_mobile(info);
    }
}

void searchers::LocalPositionCallback(const dji_sdk::LocalPosition &position)
{

}

void searchers::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if(!has_camera_info_ ){
        ROS_WARN("no camera info received yet!");
        return;
    }

    cv_bridge::CvImagePtr subscribed_color_ptr;
    try{
        subscribed_color_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if(subscribed_color_ptr->image.empty()){
        ROS_ERROR("x3 image is empty!");
        return;
    }
    cv::Mat image = subscribed_color_ptr->image;
    image.copyTo(this->global_image);

    // sample SAMPLE_NUM images to process
    if(this->filter_switcher == true){
        this->filter_counter++;
        this->img_num++;
        char buffer[100];
        sprintf(buffer,"/home/ubuntu/bagfile/original_%d.jpg",this->img_num);
        cv::imwrite(buffer,image);
        if(this->test_type != 2){
            this->get_candidate_areas(image); // BUG FIXED we must detect firstly
        }
        if(this->filter_counter == SAMPLE_NUM){
            this->filter_switcher = false;
            this->filter_counter = 0;
        }
    }

#ifdef SHOW_RESULT
    cv::imshow("X3_Image",image);
    cv::waitKey(1);
#endif
}
void searchers::InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    this->camera_info_ = (*camera_info);
    this->has_camera_info_ = true;
}


//-----------------------------------------------------------Guidance Related-----------------------------------------------------------------------//

void searchers::test_guidance()
{
    float depth_info[4] = {0.00};
    //this->process_depth_image(this->depth_down, false, depth_info);
    ROS_INFO("%f, %f, %f,%f, %f", depth_info[0],depth_info[1],depth_info[2],depth_info[3],this->obj_distance.ranges[0]);
    char str[100];
    for(int i = 0; i < 5; i++){
        sprintf(str,"depth%i",i);
        cv::imshow(str,this->depth[i]);
        cv::waitKey(1);
    }
}

bool searchers::go_down(dji_sdk::LocalPosition& tp, float yaw, float err_bound){
    return this->careful_navigation(tp, err_bound, yaw, this->go_down_time, 0, this->go_down_threshold);
}


bool searchers::back_off(dji_sdk::LocalPosition& tp, float back_dist, float yaw, float err_bound)
{
    // back off safety range 3 meters
    double roll,pitch,yaw_self;
    tf::Quaternion q(this->drone_->attitude_quaternion.q1,
                     this->drone_->attitude_quaternion.q2,
                     this->drone_->attitude_quaternion.q3,
                     this->drone_->attitude_quaternion.q0);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw_self);
    float x = tp.x - back_dist * cos(yaw_self);
    float y = tp.y - back_dist * sin(yaw_self);
    dji_sdk::LocalPosition target;
    target.x =x; target.y = y; target.z = tp.z;
    char info[100];
    sprintf(info,"back off success:%f;%f!",yaw_self,yaw);
    this->show_info(info);
    return this->careful_navigation(target, err_bound, yaw, this->back_time, GUI_REAR, this->back_threshold);
}


bool searchers::careful_navigation(const dji_sdk::LocalPosition& tp, float err_bound, float angle, int time, int direc,float thre) const
{
    dji_sdk::LocalPosition& loc = drone_->local_position;
    float x, y, z, d;
    float p = 0.7;
    for(int i = 0; i < 50 * time; i++){
        x = tp.x - loc.x;
        y = tp.y - loc.y;
        z = tp.z - loc.z;
        d = sqrt(x*x + y*y + z*z);
        if(this->ultrasonic.ranges[direc] < thre){      
            int count_temp = 0;
            for(int k = 0; k < 50; k++){
                if(this->ultrasonic.ranges[direc] < thre){
                    count_temp++;
                }
                usleep(20000);
            }
            float result_temp = float(count_temp)/50.00;
            if(result_temp > 0.4){
                ROS_INFO("careful_navigation found obstacles!");
                //return false;
            }
        }
        if(d < err_bound){
            for(int j = 0; j < 10; j++){
                drone_->local_position_control(tp.x, tp.y, tp.z, angle);
                usleep(20000);
            }
            return true;
        }
        if(d < 3){
            drone_->local_position_control(tp.x,tp.y,tp.z,angle);
            usleep(20000);
        }else{
            drone_->local_position_control(loc.x + x*p, loc.y + y*p, loc.z + z*p, angle);
            usleep(20000);
        }
    }
    return true;
}


bool searchers::search_tags(AreaType search_type)
{
    if(search_type == Area::BRIDGE){ // search three sides
        sleep(1);
        this->drone_->gimbal_angle_control(0, 200, 0, 30);
        sleep(3);
        this->drone_->gimbal_angle_control(0, 0, 0, 10);
        sleep(1);
        this->drone_->gimbal_angle_control(0, 0, 700, 20);
        sleep(2);
        this->drone_->gimbal_angle_control(0, 0, 0, 10);
        sleep(1);
        this->drone_->gimbal_angle_control(0, 0, -700, 20);
        sleep(2);
        this->drone_->gimbal_angle_control(0, 0, 0, 10);
        sleep(1);
        this->drone_->gimbal_angle_control(0, -900, 0, 10);
        sleep(1);
    }else if(search_type == Area::HOUSE){ // search four sides  
        sleep(1);
        this->drone_->gimbal_angle_control(0, 0, 0, 30);
        sleep(3);
        this->drone_->gimbal_angle_control(0, -450, 0, 10);
        sleep(1);
        this->drone_->gimbal_angle_control(0, -450, 900, 20);
        sleep(3);
        this->drone_->gimbal_angle_control(0, 0, 0, 10);
        sleep(1);
        this->drone_->gimbal_angle_control(0, -450, -900, 20);
        sleep(2);
        this->drone_->gimbal_angle_control(0, 0, 0, 10);
        sleep(1);
        this->drone_->gimbal_angle_control(0, -900, 0, 10);
        sleep(1);


    }else if(search_type == Area::WALL){ // search two sides

    }else if(search_type == Area::GROUND){ // search only one side

    }else{
        ROS_INFO("This target is not labled so far!");
        //return false;
    }
    char info_gim[] = "gimbal successful!";
    this->show_info(info_gim);
    return true;
}


// true:if arrive at target false: timeout but may not arrive at target
/// we can add a PID and the timeout (PID is shown not good)
bool searchers::local_position_navigation(const dji_sdk::LocalPosition& tp, float err_bound, float angle)
{
    dji_sdk::LocalPosition& loc = this->drone_->local_position;
    for(int i = 0; i < 50*this->local_navigation_timeout; i++){
        if(fabs(loc.x-tp.x) < err_bound && fabs(loc.y - tp.y) < err_bound && fabs(loc.z - tp.z) < err_bound){
            return true;
        }
        this->drone_->local_position_control(tp.x, tp.y, tp.z, angle);
        usleep(20000);
    }
    return false;
}

bool searchers::local_position_navigation(const dji_sdk::LocalPosition& tp, float err_bound, float angle, int time) const
{
    dji_sdk::LocalPosition& loc = drone_->local_position;
    float x, y, z, d;
    float p = 0.7;
    for(int i = 0; i < 50 * time; i++){
        x = tp.x - loc.x;
        y = tp.y - loc.y;
        z = tp.z - loc.z;
        d = sqrt(x*x + y*y + z*z);
        if(d < err_bound){
            for(int i = 0; i < 10; i++){
                drone_->local_position_control(tp.x, tp.y, tp.z, angle);
                usleep(20000);
            }
            return true;
        }
        if(d < 4){
            drone_->local_position_control(tp.x,tp.y,tp.z,angle);
            usleep(20000);
        }else{
            drone_->local_position_control(loc.x + x*p, loc.y + y*p, loc.z + z*p, angle);
            usleep(20000);
        }
    }
    return false;
}


void searchers::adjust_angle(float angle_to_go)
{
    if(angle_to_go < -180 || angle_to_go > 180){
        std::string info = "angle to go is much bigger!";
        this->drone_->send_to_mobile(info);
        usleep(20000);
        return;
    }

    std::string info = "adjust angle to specofic orientaion!";
    this->drone_->send_to_mobile(info);
    usleep(20000);

    if(angle_to_go == 0){
        double roll,pitch,yaw_self;
        tf::Quaternion q(this->drone_->attitude_quaternion.q1,
                         this->drone_->attitude_quaternion.q2,
                         this->drone_->attitude_quaternion.q3,
                         this->drone_->attitude_quaternion.q0);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw_self);
        double yaw_angle = yaw_self/PI * 180;
        if(std::fabs(yaw_angle - angle_to_go) < 5){
            return;
        }
        dji_sdk::LocalPosition local_p = this->drone_->local_position;
        float spin_bin = yaw_angle / fabs(yaw_angle);
        int total_bin  = (int)floor(fabs(yaw_angle));
        for(int i = 1; i <= total_bin; i++)
        {
            this->drone_->local_position_control(local_p.x, local_p.y, local_p.z, yaw_angle - spin_bin*i);
            usleep(20000);
        }
    }else{
        // turn 50 degree per second
        dji_sdk::LocalPosition local_p = this->drone_->local_position;
        float spin_bin = angle_to_go / fabs(angle_to_go);
        int total_bin  = (int)floor(fabs(angle_to_go));
        for(int i = 1; i <= total_bin; i++)
        {
            this->drone_->local_position_control(local_p.x, local_p.y, local_p.z, spin_bin*i);
            usleep(20000);
        }
    }

}


void searchers::locate_tar_position(float x, float y){
    float ref_x = x - this->map_region[0].x;
    float ref_y = y - this->map_region[0].y;
    float grid_pos[4][2];
    grid_pos[0][0] = ref_x; grid_pos[0][1] = ref_y - 3;
    grid_pos[1][0] = ref_x; grid_pos[0][1] = ref_y + 3;
    grid_pos[2][0] = ref_x + 3; grid_pos[2][1] = ref_y;
    grid_pos[3][0] = ref_x - 3; grid_pos[3][1] = ref_y;

    for(int i = 0; i < 4; i++){
        int x_idx = int(grid_pos[i][0]/SRH_GRID_SIZE);
        int y_idx = int(grid_pos[i][1]/SRH_GRID_SIZE);
        if(x_idx > 10){
            x_idx = 10;
        }else if(x_idx < 0){
            x_idx = 0;
        }
        if(y_idx > 10){
            y_idx = 10;
        }else if(y_idx < 0){
            y_idx = 0;
        }
        this->grid[x_idx][y_idx] = false;
    }
}

void searchers::get_bodywide_safety(int order[3])
{
    // 0 up-down; 1 forward-backward; 2 right-left;
    int direction[5];
    for (int i = 0; i < 5; i++) {
        float distance = this->obj_distance.ranges[i] - this->safety_range;
        if(distance > 0){
            direction[i] = 1; // safe
        }else{
            direction[i] = -1; // dangerous
        }
    }

    // up-down
    order[0] = direction[0];

    // front-rear
    if(direction[1] == 1 && direction[3] == -1){
        order[1] =  1; // the front is safe
    }else if( direction[1] == -1 && direction[3] == 1){
        order[1] = -1; // the rear is safe
    }else if (direction[1] == 1 && direction[3] == 1){
        order[1] = 2;  // both side is safe
    }else {
        order[1] = 0;  // both side is dangerous
    }

    // right-left
    if(direction[2] == 1 && direction[4] == -1){
        order[2] =  1; // the right is safe
    }else if( direction[2] == -1 && direction[4] == 1){
        order[2] = -1; // the left is safe
    }else if(direction[2] == 1 && direction[4] == 1){
        order[2] = 2;  // both side is safe
    }else{
        order[0] = 0;  // both side is dangerous
    }
}

bool searchers::check_back_safety(int time){
    int ultra_count = 0;
    int bumper_count = 0;
    bool ret = false;
    int order[3];
    for(int i = 1; i < 50*time;i++){
        this->get_bodywide_safety(order);
        if(order[1] > 0){
            ultra_count++;
        }
        if(this->obst_status == 0){// 0-safe 1-avoidabal
           bumper_count++;
        }
        usleep(20000);
    }

    float ration_u = float(ultra_count) / (50*time);
    float ration_b = float(bumper_count)/ (50*time);

    if(ration_u > 0.5){
        ret = true;
    }else{
        ret = true;
    }
    char info[100];
    sprintf(info,"safety ratio:%f;%f",ration_u,ration_b);
    this->show_info(info);
    return ret;
}

// turn 90 degree to scan and regard the direction of 1 at start status as zero point
bool searchers::turn_ninety_scan(int direction){
    if( !(direction == 1 || direction == -1) ){
        return false;
    }

    float threshold = 2.5;
    float angle_to_go = direction * 90.00;
    float total_bin = 90;

    float spin_bin = angle_to_go / total_bin;
    dji_sdk::LocalPosition local_p = this->drone_->local_position;
    for(int i = 1; i <= total_bin; i++)
    {
        this->drone_->local_position_control(local_p.x, local_p.y, local_p.z, spin_bin*i);
        usleep(20000);
        this->circle_distance[i-1]     = this->obj_distance.ranges[1] ;
        this->circle_distance[i-1+ 90] = this->obj_distance.ranges[2] ;
        this->circle_distance[i-1+180] = this->obj_distance.ranges[3] ;
        this->circle_distance[i-1+270] = this->obj_distance.ranges[4] ;
    }
    ROS_INFO("get distance success %f", circle_distance[260]);

    if(!lidar_data.is_open()){
        lidar_data.open(lidar_file_name,std::ios::out|std::ios::binary);
        if(!lidar_data.is_open()){
            ROS_ERROR("file write error!");
            std::string info = "file write error!";
            this->drone_->send_to_mobile(info);
            return false;
        }
    }
    char title[100];
    time_t timep;
    time(&timep);
    sprintf(title,"%s", ctime(&timep));
    lidar_data<< title << "\n";
    for(int i = 0; i < 360; i++){
        char temp[100];
        sprintf(temp,"%f", circle_distance[i]);
        lidar_data<<temp << "\n";
    }
    ROS_INFO("save success %f", circle_distance[260]);
    return true;
}


//-----------------------------------------------------------Global Tools-----------------------------------------------------------------------//

void searchers::gps_convert_ned(float &ned_x, float &ned_y,
                                double gps_t_lon, double gps_t_lat,
                                double gps_r_lon, double gps_r_lat)
{
    double d_lon = gps_t_lon - gps_r_lon;
    double d_lat = gps_t_lat - gps_r_lat;
    ned_x = DEG2RAD(d_lat) * C_EARTH;
    ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
    ROS_INFO("GPS-NED:%f-%f", gps_r_lat, gps_r_lon);
}

dji_sdk::LocalPosition searchers::gps_convert_ned(dji_sdk::GlobalPosition& loc)
{
    dji_sdk::LocalPosition local;
    gps_convert_ned(local.x, local.y,
                    loc.longitude, loc.latitude,
                    this->drone_->global_position_ref.longitude,this->drone_->global_position_ref.latitude);
    local.z = loc.height;
    return local;
}

void searchers::show_info(char* info){
    ROS_INFO("%s",info);
    std::string str(info);
    this->drone_->send_to_mobile(str);
    usleep(20000);
}

//-----------------------------------------------------------visual servoing Related-----------------------------------------------------------------------//

/// get rotated rect of each image and put them in a pool
candidates_ptr searchers::get_candidate_areas(cv::Mat& src_bgr)
{
    if(src_bgr.empty() == true){
        ROS_ERROR("get_candidate_areas:image is empty!");
        return NULL;
    }
    if(src_bgr.channels() != 3){
        ROS_ERROR("get_candidate_areas:image has not enough channels!");
        return NULL;
    }
    // get four channels: r-g-b-rgb's edges
    vector<Mat> edges;
    this->finder->get_canny_channels(src_bgr,edges);
    vector<RotatedRect> rects = this->finder->get_selected_rects(edges);

    // make a statistic calculation
    for(int i = 0; i < int(rects.size()); i++){
        if(this->filter_switcher == true)
        {
            int x_index = static_cast<int>( floor(rects[i].center.x / CELL_SIZE) );
            int y_index = static_cast<int>( floor(rects[i].center.y / CELL_SIZE) );
            int offset = x_index * GRID_HEIGHT + y_index;
            if(this->targets_ptr != NULL){
                (this->targets_ptr + offset)->push_back(rects[i]);
            }else{
                ROS_ERROR("get_candidate_areas:targets_ptr is NULL!");
                return NULL;
            }
        }
    }
    candidates_ptr rects_ptr = boost::make_shared< vector<RotatedRect> >(rects);

    char rect_num_info[100];
    sprintf(rect_num_info, "we find %d rects in this image!", int(rects_ptr->size()));
    this->show_info(rect_num_info);
    return rects_ptr;
}

bool less_than_angle(const RotatedRect& left, const RotatedRect& right)
{
    return left.angle < right.angle;
}
double accumulate_angle(double d, RotatedRect& right)
{
    return d + right.angle;
}
bool less_than_width(const RotatedRect& left,const RotatedRect& right)
{
    return (left.size.width < right.size.width);
}
// find the most robust rectangle's position and calculate their x-y
bool searchers::select_search_target(std::vector<RotatedRect> targets[][GRID_HEIGHT],std::vector<PotentialArea>& result, Point2f center_sim)
{
    if(targets == NULL){
        return false;
    }
    bool ret = false;
    //    char start[] = "get sub potential areas!\n";
    //    this->show_info(start);

    // carry on a coarse selection
    std::vector<PotentialArea> coarse_area;
    for(int i = 0; i < GRID_WIDTH; i++){
        for(int j = 0; j < GRID_HEIGHT; j++){
            std::vector<cv::RotatedRect>& rect = targets[i][j];
            if(rect.empty()){
                continue;
            }

            // 1st criterion
            float fill_ratio = (float) rect.size() / (float) SAMPLE_NUM;
            char info_ratio[50];
            sprintf(info_ratio,"the ratio is %f", fill_ratio);
            this->show_info(info_ratio);

            if (fill_ratio < 0.25){// at least there are two pictures
                continue;
            }

            // 2nd criterion remove rects with angle unstable
            float accum = 0.0;
            float angle_mean  = std::accumulate(rect.begin(), rect.end(), 0.0 , accumulate_angle) / int(rect.size());
            std::for_each(std::begin(rect), std::end(rect),[&](const RotatedRect d){
                accum  += (d.angle-angle_mean)*(d.angle-angle_mean);
            });
            float stdev = sqrt(accum/(rect.size()));
            char info_dev[50];
            sprintf(info_dev,"the std dev is %f", stdev);
            this->show_info(info_dev);

            if(stdev > 3) {// 3 Para
                continue;
            }

            // sort rects in the same cell and select the median
            std::sort(rect.begin(), rect.end(), less_than_width);
            int index = rect.size()/2;
            ret = true;

            // calculate the true distance
            float x_m, y_m;
            if(center_sim.x != 0 || center_sim.y != 0){
                float ratios = 25 * RECT_RATION_16_9;
                y_m = (rect[index].center.x - X_CENTER) / IMG_WIDTH  * ratios + center_sim.y;
                x_m = (Y_CENTER - rect[index].center.y) / IMG_HEIGHT * ratios + center_sim.x;
            }else{
                float ratios = (drone_->local_position.z - this->pixel_meter_map_err)* RECT_RATION_4_3;
                float p_x = rect[index].center.x - X_CENTER;
                float p_y = Y_CENTER - rect[index].center.y;
                float l_y = this->drone_->local_position.y;
                float l_x = this->drone_->local_position.x;
                y_m = (rect[index].center.x - X_CENTER) / IMG_HEIGHT * ratios + l_y;
                x_m = (Y_CENTER - rect[index].center.y) / IMG_HEIGHT * ratios + l_x;
                char info [200];
                sprintf(info,"px py ly lx xm ym:%f;%f;%f;%f;%f;%f",p_x,p_y,l_y,l_x,y_m,x_m);
                ROS_INFO(info);
                this->show_info(info);

                // set false flag for these areas
                this->locate_tar_position(x_m,y_m);

            }
            PotentialArea area(rect[index], x_m,y_m);
            coarse_area.push_back(area);
        }
    }

    // carry on more careful selection
    bool flag_find_similarity = false;
    for(int i = 0; i < int(coarse_area.size()-1); i++){
        flag_find_similarity = false;
        for(int j = i+1; j < int(coarse_area.size()); j++){
            float x_dis = coarse_area[i].rect.center.x - coarse_area[j].rect.center.x;
            float y_dis = coarse_area[i].rect.center.y - coarse_area[j].rect.center.y;
            float dis = std::sqrt(x_dis*x_dis + y_dis*y_dis);
            float ang = std::fabs( coarse_area[i].rect.angle - coarse_area[j].rect.angle);
            std::cout << "dis and angle:"<<dis<<";"<<ang<<std::endl;
            if(dis < 25 && ang < 10){
                flag_find_similarity = true;
                break;
            }
        }

        if(!flag_find_similarity){
            bool flag_refind =false;
            for(int k = 0; k < int(result.size()); k++){
                float x_dis = result[k].x - coarse_area[i].x;
                float y_dis = result[k].y - coarse_area[i].y;
                float dis = std::sqrt(x_dis*x_dis + y_dis*y_dis);
                if(dis < this->tar_diff_dis){// center diatance must bigger than 5 meter
                    flag_refind = true;
                    break;
                }
            }
            if(!flag_refind){
                if(coarse_area[i].rect.size.area() > 6000){
                    coarse_area[i].area_type = Area::HOUSE;
                }else{
                    coarse_area[i].area_type = Area::BRIDGE;
                }
                result.push_back(coarse_area[i]);
            }
        }
    }

    if(!coarse_area.empty()){
        bool flag_refind =false;
        for(int k = 0; k < int(result.size()); k++){
            float x_dis = result[k].x - coarse_area[int(coarse_area.size()-1)].x;
            float y_dis = result[k].y - coarse_area[int(coarse_area.size()-1)].y;
            float dis = std::sqrt(x_dis*x_dis + y_dis*y_dis);
            if(dis < this->tar_diff_dis){// center diatance must bigger than 5 meter
                flag_refind = true;
                break;
            }
        }
        if(!flag_refind){
            int index_a = int(coarse_area.size()-1);
            if(coarse_area[index_a].rect.size.area() > 6000){
                coarse_area[index_a].area_type = Area::HOUSE;
            }else{
                coarse_area[index_a].area_type = Area::BRIDGE;
            }
            result.push_back(coarse_area[index_a]);
        }
    }

    char trg_info[50];
    sprintf(trg_info,"select_search_target:%d targets detected so far!", int(result.size()));
    this->show_info(trg_info);
    return true;
}


bool searchers::get_optimal_path(const std::vector<PotentialArea>& keypoints, std::vector<PotentialArea>& path_planned)
{
    if(keypoints.empty()){
        ROS_ERROR("get_optimal_path:keypoints is empty!");
        return false;
    }

    ///**important** the curent position of keypoints is start point
    int size = keypoints.size();
    // if there are more than one targets
    std::vector<bool> flag(size,false); // mask for set elements selection
    std::vector<int>  vertex(size, 0); // selected vertexs
    std::vector<double> edges(size, 0.000); // distances betweent adjenct vertex
    flag[size-1] = true;
    vertex[0] = size-1;
    int min;
    int k;
    for (int i = 0; i < int(size -1); i++){ // control the ptr of vertex and flag
        min = INF;
        for (int j = 0; j< int(size - 1); j++){ // control the remaining elements
            if(flag[j] == false){
                Point2f a(keypoints[j].x, keypoints[j].y);
                Point2f b(keypoints[vertex[i]].x, keypoints[vertex[i]].y);
                float distance = cv::norm(a - b);
                if(distance < min){
                    min = distance;
                    k = j;// record the location of new points
                }
            }
        }
        flag[k] = true;
        edges[i] = min;
        vertex[i+1] = k;
        // generate optimal path
        path_planned.push_back(keypoints[k]);
    }
    return true;
}

// go up and fly to target one by one
int searchers::fly_to_target_point(PotentialArea& target, float height, float err_bound, int time)
{
    // if out of area constrains then give up
    int ret = 0; // do not change this value
    if(target.x > map_region[2].x || target.x < map_region[0].x
            || target.y > map_region[2].y || target.y < map_region[0].y){
        char warning[] = "a potential target is out of search area\n!";
        this->show_info(warning);
        //return -1; // change ret
    }
    dji_sdk::LocalPosition cur_pos = this->drone_->local_position;
    if(std::fabs(cur_pos.z - height) > 0.2 && cur_pos.z < height){
        this->adjust_angle(0);
        cur_pos.z = height;
        this->local_position_navigation(cur_pos, err_bound, 0, time);
    }
    cur_pos.x = target.x; cur_pos.y = target.y; cur_pos.z = height;
    if(this->local_position_navigation(cur_pos, err_bound, 0, time)){
        ret = 1;// change ret
    }
    dji_sdk::LocalPosition new_pos = this->drone_->local_position;
    char info [100];
    sprintf(info,"fly_to_target reached:%f;%f;%f",new_pos.x,new_pos.y,new_pos.z);//BUG FIXED
    this->show_info(info);
    return ret;
}


void searchers::quatToMatrix(Eigen::Matrix4d &T, tf::Quaternion &q, double x, double y, double z) {
    tf::Matrix3x3 tf_matrix = tf::Matrix3x3(q);
    T.topLeftCorner(3, 3) << tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                    tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                    tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2];
    T.col(3).head(3) << x, y, z;
    T.row(3) << 0, 0, 0, 1;
}

// adjust orientation to align with the rect
// the reference need to be a static picture
int searchers::visual_servoing_down(PotentialArea& target, float& out_yaw, float halt_height, float err_bound, cv::Point2f center_sim)
{

    bool ret = -1;
    // collect data and find centers to go
    cv::vector<cv::RotatedRect> rect_vec;
    cv::RotatedRect rect;
    cv::Point2f cur_pos;
    int count_detect = 0;
    bool flag_detect = false;
    for(int i = 0; i < ATTEMP_NUM; i++){
        // save image to sd card
        if(!this->global_image.empty()){
            this->img_num++;
            char file_name[100];
            sprintf(file_name,"/home/ubuntu/bagfile/ten_meter_%d.jpg",this->img_num);
            cv::imwrite(file_name,this->global_image);
        }else{
            char info[] = "visual servoing: mat is empty!\n";
            this->show_info(info);
        }

        // get central rect
        cur_pos = this->get_the_central_rect(this->global_image, rect);
        char info_central[100];
        sprintf(info_central, "visual servo dis-center:%d-%f;%f!",i,cur_pos.x,cur_pos.y);
        this->show_info(info_central);
        // we need confirm if this is in path planned

        // start visual servoing
        if(cur_pos.x != -1000 || cur_pos.y != -1000){// detected rects
            flag_detect = true;
            count_detect++;
            rect_vec.push_back(rect);
            if(std::fabs(cur_pos.x) < 40 && std::fabs(cur_pos.y) < 40){ // 40-1 Para
                char info[] = "visual servoing: success!\n";
                this->show_info(info);
                ret = 1;
                break;
            }else{
                float ratios = (drone_->local_position.z - this->pixel_meter_map_err)* RECT_RATION_4_3;
                float y_m =  cur_pos.x / IMG_WIDTH  * ratios + this->drone_->local_position.y;
                float x_m = -cur_pos.y / IMG_HEIGHT * ratios + this->drone_->local_position.x;
                dji_sdk::LocalPosition locp = this->drone_->local_position;
                locp.x = x_m;
                locp.y = y_m;
                locp.z = 10;
                this->local_position_navigation(locp,0.1,0,2);
                char info_success[100];
                sprintf(info_success, "visual servo:%d-%f;%f!",i,x_m,y_m);
                this->show_info(info_success);
            }
        }else{
            char info[100];
            sprintf(info, "visual servoing:in %d attemp, no rects detected!",i);
            this->show_info(info);
        }
    }

    // if all the attemp failed
    if(!flag_detect){
        return -1; // high possibility of no object
    }

    if(!rect_vec.empty()){
        int size = rect_vec.size();
        float angle_to_go = rect_vec[size-1].angle;
        if( rect_vec[size-1].size.height >  rect_vec[size-1].size.width){
            angle_to_go = ( rect_vec[size-1].angle - 90);
            char angle_info[80];
            sprintf(angle_info,"visual servoing: adjust angle to %f!",angle_to_go);
            this->show_info(angle_info);
        }
        this->adjust_angle(angle_to_go);
        // go down to halt height

        // save image to sd card
        if(!this->global_image.empty()){
            this->img_num++;
            char file_name[100];
            sprintf(file_name,"/home/ubuntu/bagfile/ten_meter_%d.jpg",this->img_num);
            cv::imwrite(file_name,this->global_image);
        }else{
            char info[] = "visual servoing: mat is empty!\n";
            this->show_info(info);
        }




        dji_sdk::LocalPosition local_p = this->drone_->local_position;
        local_p.z = halt_height;
        this->local_position_navigation(local_p, err_bound, angle_to_go, 3);
        out_yaw = angle_to_go;
        return 1;
    }
    return 0;
}


// return: the ditance in pixels to the center of the image
// return: out_rect which is the nearest one to the center
cv::Point2f searchers::get_the_central_rect(cv::Mat src_bgr, cv::RotatedRect& out_rect)
{
    candidates_ptr rects = this->get_candidate_areas(src_bgr);
    if (rects->empty()){
        return Point2f(-1000,-1000);
    }

    if(rects->size() == 1){
        out_rect = (*rects)[0];
        return Point2f((out_rect.center.x - X_CENTER), (out_rect.center.y - Y_CENTER));
    }

    // if there are rects that satisfy requirements, we get the most central one
    Point2f image_center(X_CENTER,Y_CENTER);
    float min = 2048.0;
    int k = 0;
    for(int i = 0; i < int(rects->size()); i++){
        RotatedRect temp = (*rects)[i];
        float distance = cv::norm(temp.center - image_center);
        if(distance < min){
            min = distance;
            k = i;
        }
    }
    out_rect = (*rects)[k];
    float x_pixel = (out_rect.center.x - X_CENTER);
    float y_pixel = (out_rect.center.y - Y_CENTER);
    return Point2f(x_pixel,y_pixel);
}
