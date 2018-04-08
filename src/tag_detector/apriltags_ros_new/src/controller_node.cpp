#include "controller_node.h"
#define	PI 3.1415926535
// #define CAR_HEIGH 0
// #define DEBUG

#define FLY_TO_CAR_GPS_SPEED 20

bool flag_descend = false;
bool add_abort_count = false;
bool flag_send_once = false;
bool flag_raised_to_fly_height = false;
bool flag_blind_landing = false;
bool flag_first_landing = true;


ControllerNode::ControllerNode() :
	sync_(0),
	lpf_tag_position_x_(30, 1),
	lpf_tag_position_y_(30, 1),
	lpf_tag_position_z_(30, 1),
	lpf_tag_velocity_x_(30, 1),
	lpf_tag_velocity_y_(30, 1),
	lpf_tag_velocity_z_(30, 1),
	lpf_ultrasonic_(50, 10),
	flag_landing_(false),
	is_offboard_(false),
	flag_send_(false),
	serial_same_id_num_(3),
	abort_landing_count_(0) {
	init();
}

ControllerNode::~ControllerNode() {}

void ControllerNode::init() {
	num_ = 0;

	ros::NodeHandle pnh("~");

	drone_ = new DJIDrone(nh_);

	drone_->gimbal_angle_control(0, 0, 0, 20);
	sleep(1); 	// waiting update

	pnh.getParam("position_gain/x", controller_parameters_.position_gain_.x());
	pnh.getParam("position_gain/y", controller_parameters_.position_gain_.y());
	pnh.getParam("velocity_gain/x", controller_parameters_.velocity_gain_.x());
	pnh.getParam("velocity_gain/y", controller_parameters_.velocity_gain_.y());
    //pnh.getParam("landing_start_position/x",controller_parameters_.landing_start_position.x());
    //pnh.getParam("landing_start_position/y",controller_parameters_.landing_start_position.y());
    pnh.getParam("waiting_car_longitude", waiting_car_longitude_);
    pnh.getParam("waiting_car_latitude", waiting_car_latitude_);
    pnh.getParam("static_landing_longitude", static_landing_longitude_);
    pnh.getParam("static_landing_latitude", static_landing_latitude_);
    pnh.getParam("car_height", car_height_);
    pnh.getParam("blind_landing_distance", blind_landing_distance_);

	global_dispatcher = nh_.subscribe("/task_dispatcher", 10, &ControllerNode::DispatcherCallback, this);
	// detections_sub_ = nh_.subscribe("/apriltags/detections", 1, &ControllerNode::detectionsCallback, this);
	detections_sub_.subscribe(nh_, "/apriltags/detections", 1);
	// gimbal_sub_ = nh_.subscribe("/dji_sdk/gimbal", 1, &ControllerNode::gimbalCallback, this);
	gimbal_sub_.subscribe(nh_, "/dji_sdk/gimbal", 1);
	tag_local_pub_ = nh_.advertise<LocalPosition>("/tag_local_position", 1);
	tag_global_pub_ = nh_.advertise<std_msgs::String>("/tag_global_position", 10);
//	 local_position_sub_ = nh_.subscribe("/dji_sdk/local_position", 1, &ControllerNode::localPositionCallback, this);
	local_position_sub_.subscribe(nh_, "/dji_sdk/local_position", 1);
	global_position_sub_ = nh_.subscribe("/dji_sdk/global_position", 1, &ControllerNode::globalPositionCallback, this);
	// velocity_sub_ = nh_.subscribe("/dji_sdk/velocity", 1, &ControllerNode::velocityCallback, this);
	velocity_sub_.subscribe(nh_, "/dji_sdk/velocity", 1);
	rc_sub_ = nh_.subscribe("/dji_sdk/rc_channels", 1, &ControllerNode::rcCallback, this);
	control_info_sub_ = nh_.subscribe("/dji_sdk/flight_control_info", 1, &ControllerNode::controllInfoCallback, this);
	// quat_sub_ = nh_.subscribe("/dji_sdk/attitude_quaternion", 1, &ControllerNode::quaternionCallback, this);
	quat_sub_.subscribe(nh_, "/dji_sdk/attitude_quaternion", 1);
	// tag_local_raw_pub_ = nh_.advertise<LocalPosition>("/tag_local_position_raw", 1);
	tag_velocity_pub_ = nh_.advertise<Velocity>("/tag_velocity", 1);
    ultrasonic_sub_ = nh_.subscribe("/guidance/ultrasonic", 1, &ControllerNode::obstacleCallback, this);
	data_received_from_remote_device_sub_ = nh_.subscribe("dji_sdk/data_received_from_remote_device",10,&ControllerNode::DataFromMobileCallback,this);
	lost_car_timer_ = nh_.createTimer(ros::Duration(0), &ControllerNode::TimedCommandCallback, this, true, false); //fire once when time is up

	// Synchronize image topic and plane status topics
	int queueSize_ = 8;
	sync_ = new message_filters::Synchronizer<MySyncPolicy>(
		MySyncPolicy(queueSize_),
		detections_sub_,
		gimbal_sub_,
		local_position_sub_,
		velocity_sub_,
		quat_sub_);
	sync_->registerCallback(boost::bind(&ControllerNode::syncCallBack, this, _1, _2, _3, _4, _5));
}

void ControllerNode::quatToMatrix(Eigen::Matrix4d &T, tf::Quaternion &q, double x, double y, double z) {
	tf::Matrix3x3 tf_matrix = tf::Matrix3x3(q);
	T.topLeftCorner(3, 3) << tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
					tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
					tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2];
	T.col(3).head(3) << x, y, z;
	T.row(3) << 0, 0, 0, 1;
}

void ControllerNode::gps_convert_ned(float &ned_x, float &ned_y,
									 double gps_t_lon, double gps_t_lat,
									 double gps_r_lon, double gps_r_lat) {
	double d_lon = gps_t_lon - gps_r_lon;
	double d_lat = gps_t_lat - gps_r_lat;
	ned_x = DEG2RAD(d_lat) * C_EARTH;
	ned_y = DEG2RAD(d_lon) * C_EARTH * cos(DEG2RAD(gps_t_lat));
}

LocalPosition ControllerNode::gps_convert_ned(dji_sdk::GlobalPosition loc) {
	dji_sdk::LocalPosition local;
	gps_convert_ned(local.x, local.y,
					loc.longitude, loc.latitude,
					global_position_ref_.longitude, global_position_ref_.latitude
				   );
	local.z = loc.height;
	local.header = loc.header;
	return local;
}

void ControllerNode::ned_convert_gps(double &gps_t_lon, double &gps_t_lat,
									 float ned_x, float ned_y,
									 double gps_r_lon, double gps_r_lat) {
	double d_lat = RAD2DEG(ned_x / C_EARTH);
	gps_t_lat = gps_r_lat + d_lat;
	double d_lon = RAD2DEG(ned_y / (C_EARTH * cos(DEG2RAD(gps_t_lat))));
	gps_t_lon = gps_r_lon + d_lon;
}

GlobalPosition ControllerNode::ned_convert_gps(dji_sdk::LocalPosition loc) {
	dji_sdk::GlobalPosition global;
	ned_convert_gps(global.longitude, global.latitude,
					loc.x, loc.y,
					global_position_ref_.longitude, global_position_ref_.latitude
				   );
	global.height = loc.z;
	global.altitude = global_position_ref_.altitude + loc.z;
	global.header = loc.header;
	return global;
}

LocalPosition ControllerNode::calculateLocalPosition(const apriltags::AprilTagDetection &detection) {
	tf::Quaternion q_tag;
	tf::quaternionMsgToTF(detection.pose.orientation, q_tag);
	Eigen::Matrix4d T_tag;
	quatToMatrix(T_tag, q_tag, detection.pose.position.x,
				 detection.pose.position.y, detection.pose.position.z);
	// cout << "T_tag = " << T_tag << endl;

	double roll, pitch, yaw;

	tf::Quaternion q_cam_rotation;
	q_cam_rotation.setRPY(PI / 2, 0, PI / 2);
	Eigen::Matrix4d T_cam_rotation;
	quatToMatrix(T_cam_rotation, q_cam_rotation, 0, 0, 0);
	tf::Matrix3x3(q_cam_rotation).getRPY(roll, pitch, yaw);
	// cout << "q_cam_rotation RPY = " << roll << "\t" << pitch << "\t" << yaw << endl;

	tf::Quaternion q_cam;
	q_cam.setRPY(camera_roll_ * PI / 180, camera_pitch_ * PI / 180, camera_yaw_ * PI / 180);
	Eigen::Matrix4d T_cam;
	quatToMatrix(T_cam, q_cam, x_m100_, y_m100_, -z_m100_); 	//pose of camera relative to m100 in local frame {North(x), East(y), Donw (-z)}
	//cout << "T_cam = " << T_cam << endl;
	// cout << "position of tag in camera frame : " << T_cam(0, 3) << "\t" << T_cam(1, 3) << "\t" << T_cam(2, 3) << endl;
	tf::Matrix3x3(q_cam).getRPY(roll, pitch, yaw);
	// cout << "q_cam RPY = " << roll << "\t" << pitch << "\t" << yaw << endl;
	/*
		Eigen::Matrix4d T_quad;
		quatToMatrix(T_quad, q_m100_, x_m100_, y_m100_, -z_m100_);
		// cout << "T_quad = " << T_quad << endl;
		tf::Matrix3x3(q_m100_).getRPY(roll, pitch, yaw);
		cout << "q_m100_ RPY = " << roll << "\t" << pitch << "\t" << yaw << endl;
	*/
	Eigen::Matrix4d T;
	T = /*T_quad * */T_cam * T_cam_rotation * T_tag;
	//cout << "Pose and position of tag in local frame is : " << T << endl;

	// low pass filter
	LocalPosition tag_local_position;
	tag_local_position.x = lpf_tag_position_x_.apply(T(0, 3));
	tag_local_position.y = lpf_tag_position_y_.apply(T(1, 3));
	tag_local_position.z = lpf_tag_position_z_.apply(T(2, 3));
	tag_local_position.header.stamp = detection.header.stamp;
	tag_local_position.ts = 0; 		//??? What's this?

	return tag_local_position;
}

void ControllerNode::DispatcherCallback(const dji_dispatcher::TaskDispatcher::ConstPtr &orders) {
#ifdef DEBUG
#else
	flag_start_mission_ = orders->flag_start_mission;
	flag_abort_mission_ = orders->flag_abort_mission;
	flag_abort_landing_ = orders->flag_abort_landing;
//	cout<<"flag_abort_landing = "<<flag_abort_landing_;
	flag_search_task_end_ = orders->flag_search_task_end;
	flag_auto_landing_end_ = orders->flag_auto_landing_end;
#endif
//	if(abort_landing_count_ != orders->abort_landing_count){
//        add_abort_count = true;
//		flag_landing_ = false;
//		flag_raised_to_fly_height = false;
//        flag_first_landing = false;
//		abort_landing_count_ = orders->abort_landing_count;
//	}
//	else {
//        add_abort_count = false;
//	}
}

void ControllerNode::gimbalCallback(const Gimbal::ConstPtr &gimbal_msg) {
	camera_roll_  = gimbal_msg->roll;
	camera_pitch_ = gimbal_msg->pitch;
	camera_yaw_ = gimbal_msg->yaw;
}

void ControllerNode::localPositionCallback(const LocalPosition::ConstPtr &position_msg) {
	x_m100_ = position_msg->x;
	y_m100_ = position_msg->y;
	z_m100_ = position_msg->z;
#ifdef DEBUG
	flag_start_mission_ = true;
	flag_abort_mission_ = false;
	flag_abort_landing_ = false;
	flag_search_task_end_ = true;
	flag_auto_landing_end_ = false;
#endif

    if (flag_start_mission_ && !flag_abort_mission_ /*&& !flag_abort_landing_*/ && flag_search_task_end_ && !flag_auto_landing_end_) {
        if (z_m100_ < 2 + car_height_) {
			nh_.setParam("/tag_type", "16h5");

		} else nh_.setParam("/tag_type", "36h11");
	}

	// flying to car
    if (!flag_blind_landing && !flag_landing_ && flag_start_mission_ && !flag_abort_mission_ && flag_search_task_end_) {
        nh_.setParam("/tag_type", "36h11");

		// drone_->global_position_navigation_send_request(22.535, 113.95, 10); 	// note: DO NOT FORGET TO CHANGE LATITUDE AND LONGITUDE OF CAR
		// drone_->global_position_navigation_wait_for_result (ros::Duration(20));
		// loiter until 36h11 tag is detected
		GlobalPosition car_global_position;
            //car_global_position.longitude = controller_parameters_.landing_start_position.y();//43.229710;
            //car_global_position.latitude = controller_parameters_.landing_start_position.x();//-75.421382;
        if(flag_abort_landing_){
            car_global_position.longitude = static_landing_longitude_;
            car_global_position.latitude = static_landing_latitude_;
          }
        else{
          car_global_position.longitude = waiting_car_longitude_;
          car_global_position.latitude = waiting_car_latitude_;
        }
        car_global_position.height = 13;
        LocalPosition car_local_position= gps_convert_ned(car_global_position);
        /*car_local_position.x = 0;
		car_local_position.y = 0;
        car_local_position.z = 10;*/

		double fly_height;
	if (/*abort_landing_count_ == 0 && */flag_first_landing){
			fly_height = 30;
	}else{
	    fly_height = 13;
	  }
/*		if (z_m100_ < 20 && !flag_descend) {
			drone_->velocity_control(1, velocity_x_, velocity_y_, 4, 0);

		} else if (flag_descend) {
			if (z_m100_ > 10) drone_->velocity_control(1, velocity_x_, velocity_y_, -2, 0);
*/
		if (z_m100_ < fly_height && !flag_raised_to_fly_height){
		    if(flag_first_landing)
		      drone_->attitude_control(0x08, 0, 0, 4, 0);
		    else
			drone_->velocity_control(1, velocity_x_, velocity_y_, 4, 0);			
		} else {
            flag_raised_to_fly_height = true;
            drone_->gimbal_angle_control(0, -900, 0, 10, 0);


            dji_sdk::LocalPosition loc = drone_->local_position;
            double x = car_local_position.x - loc.x;
            double y = car_local_position.y - loc.y;
            double z = car_local_position.z - loc.z;
            double d = sqrt(x*x + y*y +z*z);
            double p = 2.0/3.0;
            if(d < 2){
                drone_->local_position_control(car_local_position.x,car_local_position.y,car_local_position.z,90);
            }else{
                drone_->local_position_control(loc.x + x*p, loc.y + y*p, loc.z + z*p, 90);
            }
//             drone_->local_position_control(car_local_position.x, car_local_position.y, car_local_position.z, 0);
		}

		// lost_car_timer_.setPeriod(ros::Duration(120));
		// lost_car_timer_.start();
	}

	if (flag_start_mission_ && !flag_abort_mission_ && !flag_search_task_end_) {
		nh_.setParam("/tag_type", "25h9");
	}
/*
	if (flag_abort_landing_) {
		flag_landing_ = false;
	}
*/
	if (flag_abort_mission_) {
		// DO NOTHING, JUST HOVER, WAIT TO BE MANIPULATED
		 drone_->release_sdk_permission_control();
	}
}

void ControllerNode::globalPositionCallback(const GlobalPosition::ConstPtr &position_msg) {
	latitude_ = position_msg->latitude;
	longitude_ = position_msg->longitude;
	altitude_ = position_msg->altitude;
	height_ = position_msg->height;

	if (position_msg->ts != 0 && global_position_ref_seted_ == 0 && position_msg->latitude != 0 && position_msg->health > 3) {
		global_position_ref_ = *position_msg;
		global_position_ref_seted_ = 1;
	}
}

void ControllerNode::velocityCallback(const Velocity::ConstPtr &velocity_msg) {
	velocity_x_ = velocity_msg->vx;
	velocity_y_ = velocity_msg->vy;
	velocity_z_ = velocity_msg->vz;
}

void ControllerNode::quaternionCallback(const AttitudeQuaternion::ConstPtr &quat_msg) {
	// q0, q1, q2, q3 ->w, x, y, z
	q_m100_ = tf::Quaternion(quat_msg->q1, quat_msg->q2, quat_msg->q3, quat_msg->q0);
	double roll, pitch;
	tf::Matrix3x3(q_m100_).getRPY(roll, pitch, yaw_m100_);
}


void ControllerNode::controllInfoCallback(const FlightControlInfo::ConstPtr &info_msg) {
	has_permission_ = info_msg->serial_req_status;
}

void ControllerNode::rcCallback(const RCChannels::ConstPtr &rc_msg) {
	mode_ = rc_msg->mode;

	if (mode_ > 7000) {
		is_offboard_ = true;

		if (flag_start_mission_ && !flag_abort_mission_ && flag_search_task_end_) {
			if (has_permission_)
				return;

			drone_->request_sdk_permission_control();
			std::string info = "Start auto landing module!";
			drone_->send_to_mobile(info);
			cout << info << endl;
		}

	} else is_offboard_ = false;
}

void ControllerNode::obstacleCallback(const sensor_msgs::LaserScan::ConstPtr &obst_msg) {
	/*	if (flag_landing_ && !flag_auto_landing_end_)
			if (lpf_ultrasonic_.apply(obst_msg->ranges[0]) < 0.3)
				drone_->velocity_control(1, 0, 0, -1, 0);*/
    //object_distance_ = lpf_ultrasonic_.apply(obst_msg->ranges[0]);
    if (obst_msg->intensities[0] > 0.5 && obst_msg->ranges[0] > 0)
        object_distance_ = obst_msg->ranges[0];
    //cout << "object_distance = " << object_distance_ <<  endl;

    if(flag_blind_landing){
        //drone_->attitude_control(0x08, 0, 0, -1, 0);
        if(object_distance_ < 0.2){
            drone_->drone_disarm();
            drone_->attitude_control(0x28, 0, 0, 0, 0);
        }
        else{
            drone_->velocity_control(1, tag_velocity[0], tag_velocity[1], -1.5, 0);
//            drone_->attitude_control(0x28, 0, 0, 35, 0);
        }
    }

    if (flag_landing_ && flag_start_mission_ && !flag_abort_mission_ /*&& !flag_abort_landing_*/ && flag_search_task_end_ && !flag_auto_landing_end_) {
        if (object_distance_ <= blind_landing_distance_ /*&& sqrt(pow(position_error[0],2)+pow(position_error[1],2))<2*/) {
            flag_blind_landing = true;
            //drone_->velocity_control(1, velocity_x_, velocity_y_, -1, 0);
            //drone_->attitude_control(0x08, 0, 0, -1, 0);
            ROS_INFO("Blind landing.");
		}
        //else flag_blind_landing = false;

        if (object_distance_ <= 0.3 && !flag_send_once) {
			std::string info = "we success!!";
			drone_->send_to_mobile(info);
//			drone_->drone_disarm();
            ROS_INFO("we success!!");
			flag_send_once = true;
		}
	}
}

void ControllerNode::DataFromMobileCallback(const dji_sdk::TransparentTransmissionData &data_from_mobile_para){
    /*if (data_from_mobile_para.data[0] == '#'){
        unsigned char data[300];
		for (int i=0; i<sizeof(data_from_mobile_para.data); i++){
			data[i] = data_from_mobile_para.data[i];
		}
		// memcpy(data, data_from_mobile_para.data, (sizeof(data_from_mobile_para.data)));
		std::vector<std::string> sub_str;
		boost::split(sub_str, data, boost::is_any_of(";"));
		car_latitude_ = boost::lexical_cast<double>(sub_str[0]);
		car_longitude_ = boost::lexical_cast<double>(sub_str[1]);
                ROS_INFO("PARSED DATA:%f;%f",car_latitude_,car_longitude_);
		car_accurate_ = boost::lexical_cast<double>(sub_str[2]);
		car_speed_ = boost::lexical_cast<double>(sub_str[3]);
		cout << "car: lat " << car_latitude_ << "\tlon " << car_longitude_ << "\taccurate " << car_accurate_ << "\tspeed " << car_speed_ << endl;
    }*/
}

void ControllerNode::TimedCommandCallback(const ros::TimerEvent &e) {
	// fire once when time is up
	// climb and loiter to find car
    if (!flag_blind_landing){
        ROS_INFO("Lost the car");


        flag_landing_ = false;
        flag_raised_to_fly_height = false;
        flag_first_landing = false;

        num_ = 0;

        /* if (z_m100_ > 11) {
            drone_->velocity_control(1, velocity_x_, velocity_y_, -1, 0);

        } else {
            //draw circle
            double R = 10;
            double V = 2;
            double vx = V * sin((V / R) * ros::Time::now().toSec());
            double vy = V * cos((V / R) * ros::Time::now().toSec());
            drone_->local_position_control(x_m100_ + vx, y_m100_ + vy, 10, yaw_m100_);
            cout << "Drawing circle." << endl;
        }*/

    //	lost_car_timer_.setPeriod(ros::Duration(0.02));
    //	lost_car_timer_.start();
    }
}


void ControllerNode::detectionsCallback(const apriltags::AprilTagDetections::ConstPtr &detections_msg) {
	if (detections_msg->detections.empty())
		return;

	if (!is_offboard_)
		return;

	apriltags::AprilTagDetection detection;
	LocalPosition tag_local_position;
	GlobalPosition tag_global_position;
    //string tag_global_msg;

    if (flag_start_mission_ && !flag_abort_mission_ && !flag_search_task_end_) {
//        ROS_INFO("SEARCHING");
		    std::vector<int> v;
        for (int i = 0; i < detections_msg->detections.size(); i++) {
            v.push_back(detections_msg->detections[i].id);
        }
        last_fram_ids_.push(v);
        if(last_fram_ids_.size() != serial_same_id_num_){
            return;
        }

        // if get three
        std::map<int,int> cal;
        for(int i = 0; i < last_fram_ids_.front().size(); i++){
            cal[last_fram_ids_.front()[i]] = 1;
        }
        last_fram_ids_.pop();
        for(int i = 0; i < last_fram_ids_.front().size(); i++){
            if(cal.count(last_fram_ids_.front()[i]) == 1){
                cal[last_fram_ids_.front()[i]]++;
            }else{
                cal[last_fram_ids_.front()[i]] = 1;
            }
        }
        last_fram_ids_.pop();
        for (int i = 0; i < detections_msg->detections.size(); i++) {
            int temp = detections_msg->detections[i].id;
               if(cal.count(temp) == 1){
                   if(cal[temp] == 2){
                       tag_local_position = calculateLocalPosition(detections_msg->detections[i]);
                       tag_global_position = ned_convert_gps(tag_local_position);

                       // #;lux;luy;rux;ruy;rdx;rdy;ldx;ldy;id;latitude;longitude
                       std::string msg = "#;";
                       msg += boost::lexical_cast<std::string>(int(detections_msg->detections[i].corners2d[1].x));
                       msg+= ";";
                       msg += boost::lexical_cast<std::string>(int(detections_msg->detections[i].corners2d[1].y));
                       msg += ";";
                       msg += boost::lexical_cast<std::string>(int(detections_msg->detections[i].corners2d[0].x));
                       msg += ";";
                       msg += boost::lexical_cast<std::string>(int(detections_msg->detections[i].corners2d[0].y));
                       msg += ";";
                       msg += boost::lexical_cast<std::string>(int(detections_msg->detections[i].corners2d[3].x));
                       msg += ";";
                       msg += boost::lexical_cast<std::string>(int(detections_msg->detections[i].corners2d[3].y));
                       msg += ";";
                       msg += boost::lexical_cast<std::string>(int(detections_msg->detections[i].corners2d[2].x));
                       msg += ";";
                       msg += boost::lexical_cast<std::string>(int(detections_msg->detections[i].corners2d[2].y));
                       msg += ";";
                       msg += boost::lexical_cast<std::string>(detections_msg->detections[i].id);
                       msg += ";";
                       msg += boost::lexical_cast<std::string>(tag_global_position.latitude);
                       msg += ";";
                       msg += boost::lexical_cast<std::string>(tag_global_position.longitude);

                       tag_global_pub_.publish(msg);
                       //cout << "published tag: " << tag_global_msg << endl;
                       ROS_INFO("PUBLISH TAG");
                   }
               }
        }
        last_fram_ids_.pop();

    }

    if (!flag_blind_landing && flag_start_mission_ && !flag_abort_mission_ /*&& !flag_abort_landing_*/ && flag_search_task_end_ && !flag_auto_landing_end_) {
		nh_.getParam("/tag_type", tag_type_);

		for (int i = 0; i < detections_msg->detections.size(); i++) {
			if (tag_type_ == "36h11") {
				if (detections_msg->detections[i].id == 6) {
					detection = detections_msg->detections[i];
					flag_landing_ = true;
					flag_descend = false;
                    lost_car_timer_.setPeriod(ros::Duration(0.3));
					lost_car_timer_.start();
					break;
				}

			} else if (tag_type_ == "16h5") {
				if (detections_msg->detections[i].id == 2) {
					detection = detections_msg->detections[i];
					flag_landing_ = true;
					flag_descend = false;
                    lost_car_timer_.setPeriod(ros::Duration(0.3));
					lost_car_timer_.start();
					break;
				}

			} else cout << "tag type error !" << endl;

			if (i == detections_msg->detections.size() - 1) return;
		}

		double center_x, center_y;
		center_x = center_y = 0;

		for (int i = 0; i < 4; i++) {
			center_x += detection.corners2d[i].x;
			center_y += detection.corners2d[i].y;
		}

		center_x /= 4;
		center_y /= 4;
		// set dead zone (±30, ±20) at center of image
		double error_x = abs(320 - center_x) < 30 ? 0 : (320 - center_x);
		double error_y = abs(240 - center_y) < 20 ? 0 : (240 - center_y);
		// cout << "error_x = " << error_x << "\terror_y = " << error_y << endl;

		// 94(degree) is fov of x3, 640 is width of image
		double degree_x = -error_x * 94 / 640;
		double degree_y = error_y * 94 / 640;

		//rotation speed is about 100 degree/s
		// double time = sqrt(degree_x * degree_x + degree_y * degree_y) * 0.1;
		drone_->gimbal_angle_control(0, degree_y * 10, degree_x * 10, 10, 0);

		LocalPosition tag_local_position = calculateLocalPosition(detection);
		//cout << "camera rpy = " << camera_roll_ << "\t" << camera_pitch_ << "\t" << camera_yaw_ << endl;
		// cout << "m100 local position : " << x_m100_ << "\t" << y_m100_ << "\t" << z_m100_ << endl;
		// cout << "tag local position : " << tag_local_position.x << "\t" << tag_local_position.y << "\t" << tag_local_position.z << endl;

		tag_local_pub_.publish(tag_local_position);

		/*
			// analyze the performance of filter
			LocalPosition tag_local_position_raw;
			tag_local_position_raw.x = T(0, 3);
			tag_local_position_raw.y = T(1, 3);
			tag_local_position_raw.z = T(2, 3);
			tag_local_position_raw.header.stamp = detections_msg->header.stamp;
			tag_local_position_raw.ts = 0; 		//??? What's this?
			tag_local_raw_pub_.publish(tag_local_position_raw);
		*/
		land_on_tag(tag_local_position);
	}
}

void ControllerNode::land_on_tag(LocalPosition &tag_local_position) {
	double diff_time = (ros::Time::now().toSec() - last_roi_time_.toSec());
	last_roi_time_ = ros::Time::now();

	Eigen::Vector2d tag_position(tag_local_position.x, tag_local_position.y);
    Eigen::Vector2d tag_velocity_raw;
//    Eigen::Vector2d tag_velocity;
	Eigen::Vector2d tag_acceleration;
	/*
		if (tags_position_.size() < 10) {
			tags_position_.push_back(tag_position);
			tags_time_.push_back(ros::Time::now());
			return;
		}
		else {
			tags_position_.erase(tags_position_.begin());
			tags_position_.push_back(tag_position);
			tags_time_.erase(tags_time_.begin());
			tags_time_.push_back(ros::Time::now());
		}

		tag_velocity = (tags_position_[9] - tags_position_[0]) / (tags_time_[9] - tags_time_[0]).toSec();
		cout << "10 diff time :" << (tags_time_[9] - tags_time_[0]).toSec() << endl;
		*/

    tag_velocity_raw = (tag_position - last_position_) / diff_time;
    tag_velocity[0] = lpf_tag_velocity_x_.apply(tag_velocity_raw[0]);
    tag_velocity[1] = lpf_tag_velocity_y_.apply(tag_velocity_raw[1]);

	tag_acceleration = (tag_velocity - last_velocity_) / diff_time;
	last_velocity_ = tag_velocity;
	last_position_ = tag_position;
	// cout << "tag_velocity = " << tag_velocity << endl;

	if (num_ < 3) {
		num_++;
		return;
	}

	Velocity tag_velocity_msg;
	tag_velocity_msg.vx = tag_velocity[0];
	tag_velocity_msg.vy = tag_velocity[1];
	tag_velocity_msg.vz = 0;
	tag_velocity_msg.header.stamp = tag_local_position.header.stamp;
	tag_velocity_pub_.publish(tag_velocity_msg);

        if(flag_abort_landing_){
            position_error[0] = x_m100_ - tag_position[0];
            position_error[1] = y_m100_ - tag_position[1];
          }
        else{
            position_error[0] = x_m100_ - tag_position[0] - 0.3*tag_velocity[0];
            position_error[1] = y_m100_ - tag_position[1] - 0.3*tag_velocity[1];
          }

	//double velocity_error_x = fabs(velocity_x_ - tag_velocity[0]) > 20 ? (20 * SIGN(velocity_x_ - tag_velocity[0])) : (velocity_x_ - tag_velocity[0]);
	//double velocity_error_y = fabs(velocity_y_ - tag_velocity[1]) > 20 ? (20 * SIGN(velocity_y_ - tag_velocity[1])) : (velocity_y_ - tag_velocity[1]);
	double velocity_error_x = velocity_x_ - tag_velocity[0];
	double velocity_error_y = velocity_y_ - tag_velocity[1];
	Eigen::Vector2d velocity_error(velocity_error_x, velocity_error_y);
	Eigen::Vector2d acceleration;
	/*
	acceleration = -position_error.cwiseProduct(controller_parameters_.position_gain_)
	               - velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)
	               + tag_acceleration;

	velocity_x_ += acceleration[0];
	velocity_y_ += acceleration[1];
	drone_->velocity_control(1, velocity_x_, velocity_y_, 0, 0);
	*/
	Eigen::Vector2d velocity;
	velocity = -position_error.cwiseProduct(controller_parameters_.position_gain_)
                - velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)
			   + tag_velocity;

	double landing_velocity;

    if (object_distance_ > blind_landing_distance_) {
        if (z_m100_ < 1.5 + car_height_) {	// note: DO NOT FORGET TO ADD HIGH OF CAR
            landing_velocity = -0.2;

			if (flag_send_ == false) {
				std::string info = "Ready to land!";
				drone_->send_to_mobile(info);
                ROS_INFO("Lower than 1.5m");
				flag_send_ = true;
			}

        } else if (z_m100_ < 3 + car_height_) {
//            if(sqrt(pow(position_error[0],2)+pow(position_error[1],2))<1.2)
              landing_velocity = -0.3;
//            else landing_velocity = 0;

        } else landing_velocity = -0.4;

//        double dist = sqrt(pow(position_error[0],2) + pow(position_error[1], 2));
        //double yaw_error = RAD2DEG(atan2(-position_error[1], -position_error[0]));
//        double yaw_error = 90 - yaw_m100_;
//        double yaw_rate = (fabs(yaw_error) >  5 && dist > 2) ? SIGN(yaw_error) * 30 : 0;
//        double yaw_rate = 90;
        drone_->velocity_control(1, velocity[0], velocity[1], landing_velocity, 0);
	};
}

void ControllerNode::syncCallBack(const apriltags::AprilTagDetections::ConstPtr &detections_msg,
								  const Gimbal::ConstPtr &gimbal_msg,
								  const LocalPosition::ConstPtr &position_msg,
								  const Velocity::ConstPtr &velocity_msg,
								  const AttitudeQuaternion::ConstPtr &quat_msg) {
	gimbalCallback(gimbal_msg);
	localPositionCallback(position_msg);
	velocityCallback(velocity_msg);
	quaternionCallback(quat_msg);
	detectionsCallback(detections_msg);
}

bool ControllerNode::local_position_navigation(const dji_sdk::LocalPosition& tp, float err_bound, float angle, float time)
{
    bool ret = true;

    for(int i = 0; i < 50 * time; i++){
        dji_sdk::LocalPosition loc = drone_->local_position;
        double x = tp.x - loc.x;
        double y = tp.y - loc.y;
        double z = tp.z - loc.z;
        double d = sqrt(x*x + y*y +z*z);
        if(d < err_bound){
            for(int i = 0; i < 10; i++){
                drone_->local_position_control(tp.x, tp.y, tp.z, angle);
                usleep(20000);
            }
            return true;
        }

        double p = 0.5;
        if(d < 2){
            drone_->local_position_control(tp.x,tp.y,tp.z,angle);
            usleep(20000);
        }else{
            drone_->local_position_control(loc.x + x*p, loc.y + y*p, loc.z + z*p, angle);
            usleep(20000);
        }
        usleep(20000);
    }
    return ret;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "controller_node");

	ControllerNode *controller_node = new ControllerNode();

	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();

	//clear
	delete controller_node;
	controller_node = NULL;

	return 0;
}
