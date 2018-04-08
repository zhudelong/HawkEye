#ifndef _SEARCHERS_H_
#define _SEARCHERS_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <dji_sdk/dji_drone.h>
#include <stdlib.h>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include <dji_sdk/dji_sdk_node.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>
#include <dji_dispatcher/TaskDispatcher.h>
#include <dji_dispatcher/TaskState.h>
#include <queue>          // std::queue
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <boost/lexical_cast.hpp>

// for 3*3*2.8 and 4*3.5*2.8
#define MAP_HEIGHT (float) 20.00
#define FLY_HEIGHT (float) 10.00
#define HAT_HEIGHT (float) 4.500
#define SUB_AREAS_NUM 4
//#define WEB_CAMERA
//#define OFFLINE_TEST_CV
#define OFFLINE_TEST_CT
//#define GUIDANCE_TEST
// #define MAIN_LOOP

#define RECT_RATION_16_9 (double) 0.816236674
#define RECT_RATION_4_3  (double) 1.24286244

// x3 image
#define X_CENTER 320
#define Y_CENTER 240
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define CANNY_THRES 100

// depth image
#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240

// map
#define CELL_SIZE 20
#define SAMPLE_NUM 20
#define IMG_GROUP_NUM 5
#define INF (~(0x1<<31))        // infinite (0X7FFFFFFF)

// control
#define ERR_BOUND 0.1
#define YAW_BOUND 0.1

#define SRH_HEIGHT 1.5

#define MANIFOLD
#define HIT_TEST


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

class Corners{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Corners():
        start_point(0.0,0.0),
        cross_point(0.0,0.0),
        left_point(0.0,0.0),
        right_point(0.0,0.0){}

    Eigen::Vector2d start_point;
    Eigen::Vector2d cross_point;
    Eigen::Vector2d left_point;
    Eigen::Vector2d right_point;
};

typedef boost::shared_ptr<std::vector<cv::RotatedRect> > candidates_ptr;
typedef boost::shared_ptr< std::vector<PotentialArea> > pathpoints_ptr;
typedef boost::shared_ptr<cv::Mat> cvMat_ptr;

class searchers
{
public:

    // global dispatchers' orders
    static bool flag_start_mission;
    static bool flag_abort_mission;
    static bool flag_abort_landing;
    static bool flag_search_task_end;
    static bool flag_auto_landing_end;
    bool send_rect_to_mobile(cv::Point2f points[]);
    ros::Publisher  task_state_publisher;

    int img_num;


private:  
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
    ros::Subscriber guidance_ultrasonic_subscriber;
    ros::Subscriber depth_image0_subscriber;
    ros::Subscriber depth_image1_subscriber;
    // subscribe msg from x3_camera node
    ros::Subscriber x3_info_subscriber;
    image_transport::Subscriber x3_image_subscriber;

public:
    bool filter_switch;
    int  filter_counter;
    DJIDrone* drone_;
    Corners corners;
    Eigen::Vector2d map_anchor[4];
    std::vector<cv::RotatedRect>* targets_ptr;
    bool get_sub_potential_areas(std::vector<cv::RotatedRect> targets[32][24], std::vector<PotentialArea> &result,
                                 cv::Point2f center_sim = cv::Point2f(0.0, 0.0));

    // variables for all the callbacks
    cv::Mat global_image;
    float tuned_yaw;
    bool  has_camera_info_;
    bool  is_running_;


public:/// ros node related
    searchers(ros::NodeHandle& nh, ros::NodeHandlePtr node_ptr);
    ~searchers();
    void init_parameters(ros::NodeHandlePtr node_para);
    void setup_drone_subscriber(ros::NodeHandle& nh);
    void DispatcherCallback(const dji_dispatcher::TaskDispatcher& orders);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
    void GlobalPositionCallback(const dji_sdk::GlobalPosition& position);
    void LocalPositionCallback(const dji_sdk::LocalPosition& position);
    void GuidanceDistanceCallback(const sensor_msgs::LaserScan& obj_dis);
    void GuidanceUltrasonicCallback(const sensor_msgs::LaserScan& ultra_dis);
    void GuidanceDepthImageCallback1(const sensor_msgs::ImageConstPtr& depth_img1);
    void GuidanceDepthImageCallback0(const sensor_msgs::ImageConstPtr& depth_img0);


public:/// X3 camera related
    cv::RotatedRect start_area;
    candidates_ptr possible_areas;
    pathpoints_ptr path_keypoints;

    // halt on the top of search areas
    candidates_ptr get_candidate_areas(cv::Mat &src_gray);
    bool local_position_navigation(const dji_sdk::LocalPosition &p, float err_bound, float angle = 0.00);
    cv::Point2f get_the_central_rect(cv::Mat& src_gray, cv::RotatedRect& out_rect);
    pathpoints_ptr get_path_ponits(candidates_ptr points, cv::Point2f start_point, int sp_locations);
    bool get_optimal_path(const std::vector<PotentialArea> & kaypoints, std::vector<PotentialArea>& path_planned);
    bool fly_to_target_point(PotentialArea &target, float height, float err_bound);
    int  adjust_oritation(cv::Mat& target, float fly_height, float halt_height, float err_bound, cv::Point2f center_sim = cv::Point2f(0.0, 0.0));

    // automatically search the target areas
    bool search_tags(AreaType search_type);
    bool search_bridge(PotentialArea& target);
    void search_house(PotentialArea& target);
    void search_wall(PotentialArea& target);
    void search_ground(PotentialArea& target);

public: /// guidance related
    sensor_msgs::LaserScan ultrasonic;
    sensor_msgs::LaserScan obj_distance;
    cv::Mat depth_front;
    cv::Mat depth_down;

    void test_guidance();
    int process_depth_image(cv::Mat &image, bool left_right, float out_ret[]);
    bool back_off(dji_sdk::LocalPosition& localp, float back_dist, float yaw, float err_bound);







#ifdef ADVANCED
    bool search_bridge_highweight(PotentialArea& target);
    void main_loop();
    std::list<cv::RotatedRect> rect_group;
    int get_stable_central_rect(std::list<cv::RotatedRect>& rect_group, cv::RotatedRect& out_rect);
    bool step_to_the_center( cv::Point2f tar_position);
    void quatToMatrix(Eigen::Matrix4d &T, tf::Quaternion &q, double x, double y, double z);
    bool fly_to_start_point(pathpoints_ptr path, int location, cv::Mat &src_cur, cv::Mat &src_prv); // servo to specific height
#endif


};



#endif

