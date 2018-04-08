#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <ros/ros.h>
// #include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <dji_sdk/dji_sdk_node.h>
#include <apriltags/AprilTagDetections.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "LowPassFilter2p.hpp"
#include "WeightedAverageFilter.h"
#include <dji_dispatcher/TaskDispatcher.h>
#include <queue>

#define C_EARTH (double) 6378137.0
#define C_PI (double) 3.141592653589793
#define DEG2RAD(DEG) ((DEG)*((C_PI)/(180.0)))
#define RAD2DEG(RAD) ((RAD)*((180.0)/(C_PI)))
#define SIGN(figure) ((figure) >= 0 ? 1 : -1)

using namespace std;
using namespace DJI::onboardSDK;
using namespace dji_sdk;


class ControllerParameters {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ControllerParameters()
		: position_gain_(Eigen::Vector2d(0.5, 0.5)),
		  velocity_gain_(Eigen::Vector2d(0.5, 0.5)),
		  attitude_gain_(Eigen::Vector3d(0.5, 0.5, 0.5)),
		  angular_rate_gain_(Eigen::Vector3d(0.5, 0.5, 0.5)),
	landing_start_position(Eigen::Vector2d(22.59193162, 113.9655637)){}

	Eigen::Vector2d position_gain_;
	Eigen::Vector2d velocity_gain_;
	Eigen::Vector3d attitude_gain_;
	Eigen::Vector3d angular_rate_gain_;
        Eigen::Vector2d landing_start_position;
};

class ControllerNode {
public:
	ControllerNode();
	~ControllerNode();
	void init();

	ControllerParameters controller_parameters_;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	ros::NodeHandle nh_;

	std::string task_;
	std::string tag_type_;

	math::LowPassFilter2p lpf_tag_position_x_;
	math::LowPassFilter2p lpf_tag_position_y_;
	math::LowPassFilter2p lpf_tag_position_z_;
	math::LowPassFilter2p lpf_tag_velocity_x_;
	math::LowPassFilter2p lpf_tag_velocity_y_;
	math::LowPassFilter2p lpf_tag_velocity_z_;

	math::LowPassFilter2p lpf_ultrasonic_;

	math::WeightedAverageFilter waf_tag_position_x_;
	math::WeightedAverageFilter waf_tag_position_y_;
	math::WeightedAverageFilter waf_tag_position_z_;

	vector<Eigen::Vector2d> tags_position_;
	vector<ros::Time> tags_time_;

	// ROSAdapter *rosAdapter_;
	DJIDrone* drone_;

	ros::Subscriber global_dispatcher;
	// ros::Subscriber detections_sub_;
	// ros::Subscriber gimbal_sub_;
//	 ros::Subscriber local_position_sub_;
	ros::Subscriber global_position_sub_;
	// ros::Subscriber velocity_sub_;
	ros::Subscriber rc_sub_;
	ros::Subscriber control_info_sub_;
	// ros::Subscriber quat_sub_;
	ros::Subscriber ultrasonic_sub_;
	ros::Subscriber data_received_from_remote_device_sub_;

	message_filters::Subscriber<apriltags::AprilTagDetections> detections_sub_;
	message_filters::Subscriber<dji_sdk::Gimbal> gimbal_sub_;
	message_filters::Subscriber<dji_sdk::LocalPosition> local_position_sub_;
	// message_filters::Subscriber<dji_sdk::GlobalPosition> global_position_sub_;
	message_filters::Subscriber<dji_sdk::Velocity> velocity_sub_;
	// message_filters::Subscriber<dji_sdk::RCChannels> rc_sub_;
	// message_filters::Subscriber<dji_sdk::FlightControlInfo> control_info_sub_;
	message_filters::Subscriber<dji_sdk::AttitudeQuaternion> quat_sub_;

	typedef message_filters::sync_policies::ApproximateTime<apriltags::AprilTagDetections, Gimbal, LocalPosition, Velocity, AttitudeQuaternion> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> * sync_;

	ros::Timer lost_car_timer_;

	bool flag_start_mission_;
	bool flag_abort_mission_;
	bool flag_abort_landing_;
	bool flag_search_task_end_;
	bool flag_auto_landing_end_;
	int abort_landing_count_;
	bool flag_landing_;
	bool flag_send_;

	ros::Publisher tag_local_pub_;
	ros::Publisher tag_global_pub_;
	ros::Publisher tag_local_raw_pub_;
	ros::Publisher tag_velocity_pub_;

	bool has_permission_;
	double mode_;
	bool is_offboard_;
	int id_;

    double blind_landing_distance_;
    double car_height_;
    double waiting_car_longitude_;
    double waiting_car_latitude_;
    double static_landing_longitude_;
    double static_landing_latitude_;

	int num_;
	ros::Time last_roi_time_;
	Eigen::Vector2d last_position_;
	Eigen::Vector2d last_velocity_;
    std::queue< std::vector<int> > last_fram_ids_;
    //std::<std::vector<int>> last_fram_ids_;
	int serial_same_id_num_;

	double camera_roll_;
	double camera_pitch_;
	double camera_yaw_;

	double x_m100_;
	double y_m100_;
	double z_m100_;
	tf::Quaternion q_m100_;
	double yaw_m100_;

    Eigen::Vector2d position_error;
    Eigen::Vector2d tag_velocity;

	double velocity_x_;
	double velocity_y_;
	double velocity_z_;

	double latitude_;
	double longitude_;
	double altitude_;
	double height_;

	double object_distance_;

	double car_latitude_;
	double car_longitude_;
	double car_accurate_;
	double car_speed_;

	GlobalPosition global_position_ref_;	// home
	int global_position_ref_seted_ = 0;

	void DispatcherCallback(const dji_dispatcher::TaskDispatcher::ConstPtr& orders);
	void quatToMatrix(Eigen::Matrix4d &T, tf::Quaternion &q, double x, double y, double z);
	void detectionsCallback(const apriltags::AprilTagDetections::ConstPtr& detections_msg);
	void gimbalCallback(const Gimbal::ConstPtr& gimbal_msg);
	void localPositionCallback(const LocalPosition::ConstPtr& position_msg);
	void globalPositionCallback(const GlobalPosition::ConstPtr& position_msg);
	void velocityCallback(const Velocity::ConstPtr& velocity_msg);
	void quaternionCallback(const AttitudeQuaternion::ConstPtr& quat_msg);
	void rcCallback(const RCChannels::ConstPtr& rc_msg);
	void controllInfoCallback(const FlightControlInfo::ConstPtr& info_msg);
	void obstacleCallback(const sensor_msgs::LaserScan::ConstPtr& obst_msg);
	void TimedCommandCallback(const ros::TimerEvent &e);
	
	void syncCallBack(const apriltags::AprilTagDetections::ConstPtr& detections_msg,
	                  const Gimbal::ConstPtr& gimbal_msg,
			  const LocalPosition::ConstPtr& position_msg,
	                  const Velocity::ConstPtr& velocity_msg,
	                  const AttitudeQuaternion::ConstPtr& quat_msg);

	void gps_convert_ned(float &ned_x, float &ned_y, double gps_t_lon, double gps_t_lat, double gps_r_lon, double gps_r_lat);
	LocalPosition gps_convert_ned(dji_sdk::GlobalPosition loc);
	void ned_convert_gps(double &gps_t_lon, double &gps_t_lat, float ned_x, float ned_y, double gps_r_lon, double gps_r_lat);
	GlobalPosition ned_convert_gps(dji_sdk::LocalPosition loc);
	LocalPosition calculateLocalPosition(const apriltags::AprilTagDetection& detection);
	void DataFromMobileCallback(const dji_sdk::TransparentTransmissionData &data_from_mobile_para);
    bool local_position_navigation(const dji_sdk::LocalPosition& tp, float err_bound, float angle, float time);

	void land_on_tag(LocalPosition& tag_local_position);
};
