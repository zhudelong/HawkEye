#ifndef _SEARCHERS_H_
#define _SEARCHERS_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dji_sdk/dji_drone.h>
#include <queue>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <dji_sdk/dji_sdk_node.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>
#include <dji_dispatcher/TaskDispatcher.h>
#include <dji_dispatcher/TaskState.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/lexical_cast.hpp>
#include <std_msgs/String.h>
#include <opencv2/gpu/gpu.hpp>
#include "TargetFinder.h"
#include <dji_bumper/DetectionResult.h>


#define SRH_GRID_SIZE 5
#define SIGN(figure) ((figure) >= 0 ? 1 : -1)
#define SUB_AREAS_NUM 4
//#define WEB_CAMERA
//#define OFFLINE_TEST_CV
//#define OFFLINE_TEST_CT
//#define GUIDANCE_TEST
#define MAIN_LOOP
//#define OBJ_AVOID

// for 3*2.5*2.5 and 5*5*4
#define PI 3.41592653
#define RECT_RATION_16_9 (double) 0.816236674
#define RECT_RATION_4_3  (double) 1.24286244  //1.248502984

// x3 image
#define X_CENTER 320
#define Y_CENTER 240
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define CANNY_THRES 100

// depth image
#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240
#define GUI_DOWN 0
#define GUI_REAR 3

// map
#define GRID_WIDTH  16
#define GRID_HEIGHT 12
#define CELL_SIZE 40

#define SAMPLE_NUM 3
#define ATTEMP_NUM 3
#define IMG_GROUP_NUM 5
#define INF (~(0x1<<31))        // infinite (0X7FFFFFFF)

// control
#define ERR_BOUND 0.2 // the most robust
#define MAP_BOUND 0.1
#define MANIFOLD
#define HIT_TEST
//#define SHOW_RESULT

typedef enum Area{NOTHING, BRIDGE, HOUSE, WALL, GROUND} AreaType;

class PotentialArea
{
public:
    PotentialArea():x(0.0),y(0.0),area_type(Area::NOTHING),is_covered(false){}
    PotentialArea(cv::RotatedRect &rect_para):
        rect(rect_para),x(0.0),y(0.0),area_type(Area::NOTHING),is_covered(false){}
    PotentialArea(cv::RotatedRect &rect_para, float x_para, float y_para):
        rect(rect_para),x(x_para),y(y_para),area_type(Area::NOTHING),is_covered(false){}
    bool is_covered;
    AreaType area_type;
    cv::RotatedRect rect;
    float x, y; // local positions
};

typedef boost::shared_ptr<std::vector<cv::RotatedRect> > candidates_ptr;
typedef boost::shared_ptr< std::vector<PotentialArea> > pathpoints_ptr;
typedef boost::shared_ptr<cv::Mat> cvMat_ptr;

class searchers
{
public:/// temp test use
    dji_sdk::LocalPosition test_anchor[4];



public:/// global parameters
    int test_type;        // measure
    int grid_x_idx;      // x-index
    int grid_y_idx;      // y-index
    double anchor_center_dis;   // distance between 2 anchors
    double center_bia_x; // map center offset x
    double center_bia_y; // map center offset y
    double tar_diff_dis; // distance between 2 detected targets

    double pixel_meter_map_err; // error for pixel to meter
    int local_navigation_timeout;// force navigation timeout
    double drone_speed; // search speed
    double drone_height; // search height
    double safety_range; // guidance safety_range
    double area_width;   // width of map grids

    // for node usage
    int take_off_time;
    int go_up_time;
    int go_up_speed;
    int fly_to_target_time;
    int fly_to_anchor_time;
    int back_time;
    double back_distance;
    double back_threshold;
    int go_down_time;
    double go_down_threshold;

    // key parameter
    double map_height;
    double fly_height;
    double halt_height;
    double srh_height;

    // flag
    int height_status;
    int layout_status;

    int right_fly_time;


public:/// global dispatchers' orders
    static bool flag_start_mission;
    static bool flag_abort_mission;
    static bool flag_abort_landing;
    static bool flag_search_task_end;
    static bool flag_auto_landing_end;
    static bool flag_detect_all;
    static bool flag_limitation_ok;
    static bool flag_go_back;
    static int  abort_landing_count;
    static int  status_code;


public:
    ros::NodeHandlePtr node_;
    boost::shared_ptr<image_transport::ImageTransport> image_;  
    sensor_msgs::CameraInfo camera_info_;
    // global dispatcher
    ros::Subscriber global_dispatcher;
    // subscribe msg from dji_auto_landing node
    ros::Subscriber tag_local_position_subscriber;
    ros::Subscriber tag_global_position_subscriber;
    // subscribe msg from dji_guidance node
    ros::Subscriber guidance_distance_subscriber;
    ros::Subscriber guidance_bumper_subscriber;
    ros::Subscriber guidance_ultrasonic_subscriber;
    // subscribe msg from x3_camera node
    ros::Subscriber x3_info_subscriber;
    image_transport::Subscriber x3_image_subscriber;
    ros::Publisher  task_state_publisher;

public:
    bool tag_sender_switcher;
    bool filter_switcher;
    int  filter_counter;
    DJIDrone* drone_;
    TargetFinder* finder;
    bool grid[10][10];
    dji_sdk::GlobalPosition corners[4];
    dji_sdk::LocalPosition map_region[4];
    dji_sdk::LocalPosition map_anchor[4];

    std::vector<cv::RotatedRect>* targets_ptr;
    std::ofstream lidar_data;
    char lidar_file_name[100];
    // variables for all the callbacks
    cv::Mat global_image;
    float tuned_yaw;
    bool  has_camera_info_;
    bool  is_running_;


public:/// ros node related
    searchers(ros::NodeHandle& nh, ros::NodeHandlePtr node_ptr);
    ~searchers();
    void init_parameters(ros::NodeHandlePtr node_para);
    void init_map_regions();
    void DispatcherCallback(const dji_dispatcher::TaskDispatcher& orders);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
    void GlobalPositionCallback(const std_msgs::String &position);
    void LocalPositionCallback(const dji_sdk::LocalPosition& position);
    void GuidanceBumperCallback(const dji_bumper::DetectionResult& obstacle);
    void GuidanceDistanceCallback(const sensor_msgs::LaserScan& obj_dis);
    void GuidanceUltrasonicCallback(const sensor_msgs::LaserScan& ultra_dis);
    void GuidanceDepthImageCallback4(const sensor_msgs::ImageConstPtr& depth_img);
    void GuidanceDepthImageCallback3(const sensor_msgs::ImageConstPtr& depth_img);
    void GuidanceDepthImageCallback2(const sensor_msgs::ImageConstPtr& depth_img);
    void GuidanceDepthImageCallback1(const sensor_msgs::ImageConstPtr& depth_img);
    void GuidanceDepthImageCallback0(const sensor_msgs::ImageConstPtr& depth_img);

public: /// guidance related
    // perception input
    cv::Mat depth[5];
    sensor_msgs::LaserScan ultrasonic;
    sensor_msgs::LaserScan obj_distance;
    float point_distance[5];
    float circle_distance[360];
    float obst_x,obst_y,obst_z;
    unsigned char obst_status;

    // global tools
    void show_info(char* info);
    dji_sdk::LocalPosition gps_convert_ned(dji_sdk::GlobalPosition &loc);
    void gps_convert_ned(float &ned_x, float &ned_y,
                         double gps_t_lon, double gps_t_lat,
                         double gps_r_lon, double gps_r_lat);
    // multi-info processing tools
    bool turn_ninety_scan(int direction);
    void get_bodywide_safety(int order[]);
    bool check_back_safety(int time);
    void locate_tar_position(float x, float y );

    // control policies
    void test_guidance();
    void adjust_angle(float angle_to_go);
    bool go_down(dji_sdk::LocalPosition& tp, float yaw, float err_bound);
    bool back_off(dji_sdk::LocalPosition& tp, float back_dist, float yaw, float err_bound);
    bool local_position_navigation(const dji_sdk::LocalPosition &p, float err_bound, float angle);
    bool local_position_navigation(const dji_sdk::LocalPosition& tp, float err_bound, float angle , int time) const;
    bool careful_navigation(const dji_sdk::LocalPosition& tp, float err_bound, float angle, int time, int direc, float thre) const;
    bool search_tags(AreaType search_type);


public:/// X3 camera related
    int img_num; // control the image writing
    bool gpu_enabled;
    cv::RotatedRect start_area;
    candidates_ptr possible_areas;
    pathpoints_ptr path_keypoints;
    // halt on the top of search areas
    candidates_ptr get_candidate_areas(cv::Mat &src_bgr);
    cv::Point2f get_the_central_rect(Mat src_bgr, cv::RotatedRect& out_rect);
    pathpoints_ptr get_path_ponits(candidates_ptr points, cv::Point2f start_point, int sp_locations);
    bool get_optimal_path(const std::vector<PotentialArea> & kaypoints, std::vector<PotentialArea>& path_planned);
    int fly_to_target_point(PotentialArea &target, float height, float err_bound, int time);
    void quatToMatrix(Eigen::Matrix4d &T, tf::Quaternion &q, double x, double y, double z);
    int  visual_servoing_down(PotentialArea &target, float &out_yaw, float halt_height, float err_bound,
                          cv::Point2f center_sim = cv::Point2f(0.0, 0.0));
    bool select_search_target(std::vector<cv::RotatedRect> targets[][GRID_HEIGHT], std::vector<PotentialArea> &result,
                                 cv::Point2f center_sim = cv::Point2f(0.0, 0.0));
};

#endif

