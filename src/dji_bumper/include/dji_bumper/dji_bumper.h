#ifndef DJI_BUMPER_H
#define DJI_BUMPER_H

#include <cv_bridge/cv_bridge.h>
#include "dji_bumper/dji_scissor.h"
#include "dji_bumper/DetectionResult.h"
#include "dji_bumper/SwitchMode.h"
#include <DJI_guidance.h>
#include <DJI_utility.h>
#include <fstream>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>
/*
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
*/
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <std_msgs/UInt8.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#define USER_NAME "ubuntu"  // !! remember to adjust this value when debug on different platform
#define DATA_PACK_ID_FOR_SAVING 9 // 0 - 9
#define DATA_PACK_ID_FOR_LOADING 6 // 0 - 9

//#define OFFLINE_DEBUG

//#define DISPLAY_GRAY_IMAGE
//#define DISPLAY_DISPARITY_IMAGE
//#define DISPLAY_DEPTH_IMAGE
//#define DISPLAY_OBSTACLE_DISTANCE
//#define DISPLAY_ULTRASONIC
//#define DISPLAY_LOADED_IMAGE
//#define DISPLAY_MATCH_RESULT
//#define DISPLAY_TRIANULATION_RESULT
//#define DISPLAY_SEGMENTATION_RESULT
//#define DISPLAY_LAYER
//#define DISPLAY_PATH

//#define SAVE_LEFT_IMAGE
//#define SAVE_RIGHT_IMAGE
//#define SAVE_DISPARITY_IMAGE
//#define SAVE_DEPTH_IMAGE
//#define SAVE_CALI
//#define SAVE_VELOCITY
//#define SAVE_MOTION
//#define SAVE_IMU
//#define SAVE_MATCH_RESULT
//#define SAVE_TRIANGULATION_RESULT
//#define SAVE_HIST

//#define PUBLISH_DISPARITY_IMAGE
//#define PUBLISH_DEPTH_IMAGE
#define PUBLISH_OBSTACLE_DISTANCE
#define PUBLISH_ULTRASONIC
//#define PUBLISH_MOTION
//#define PUBLISH_IMU
#define PUBLISH_DETECTION_RESULT

/**
 * image type
 */
typedef enum e_img_type
{
  img_type_grey = 0,
  img_type_disparity = 1,
  img_type_depth = 2
} img_type_t;


/**
 * different mode for stereo matching
 */
typedef enum e_match_mode
{
  match_mode_BM = 0,
  match_mode_BP = 1,
  match_mode_CSBP = 2,
  match_mode_BM_CPU = 3,
  match_mode_SGBM_CPU = 4
} match_mode_t;

/**
 * judgement of path detection in each layer
 */
typedef enum e_layer_prop
{
  layer_prop_safe = 0,
  layer_prop_avoidable = 1,
  layer_prop_blocked = 2
} layer_prop_t;

/**
 * virtual bumper
 */
class djiBumper
{
private:
  ros::NodeHandle nh_;

  int image_width_;   // width of original image
  int image_height_;  // height of original image
  int image_size_;    // size of original image

  float max_depth_;   // ignore points that are further than max_depth_ (meters)
  float min_depth_;   // ignore points thar are closer than min_depth_ (meter)

  float path_width_;  // safe width of path for UAV to go through
  float path_height_; // safe height of path for UAV to go through
  float path_tolerance_ratio_;

  ros::Publisher pub_obj_dis;
  ros::Publisher pub_ultrasonic;
  ros::Publisher pub_img_disp[5];
  ros::Publisher pub_img_depth[5];
  ros::Publisher pub_imu;
  ros::Publisher pub_detection_result_;

  ros::ServiceServer srv_mode;

  static int my_callback(int data_type, int data_len, char* content); // data callback
  bool switchModeCb(dji_bumper::SwitchMode::Request& req, dji_bumper::SwitchMode::Response& res);

  int checkLayer(cv::Mat& layer, int& window_width, int& window_height, int& pt_thres, layer_prop_t& judge, float& offset_x, float& offset_y);

  djiScissor scissor; // a instance of image segmentation tool. TO BE COMPLETED !!!

public:
  djiBumper();
  ~djiBumper();

  int init(void); // init guidance
  int load(int id = 1, img_type_t type = img_type_grey, int frame = 0, std::string img_dir = std::string("/home/") + USER_NAME + "/data_pack/" + char('0' + DATA_PACK_ID_FOR_LOADING) + "/img/"); // load image for off-line debug
  int match(int id = 5, int ndisparities = 64, int winSize = 5, match_mode_t mode = match_mode_BM);  // get disparity
  int triangulate(int id = 5); // get depth
//  int generateCloud(int id=5);  // generate point cloud from detph image
  int segment(int id = 5);  // segment image
  int detectPath(int id, int layer_num, float& safe_dist, float& offset_x, float& offset_y); // find a path to go through
  void publish(void);  // publish ROS message

  unsigned char checkSafetyByVision(float threshold_h = 1.5, float threshold_l = 0.5, int threshold_cnt = 25);
  unsigned char checkSafetyByUltrasonic(float threshold_h = 1.5, float threshold_l = 0.5);
};

#endif

