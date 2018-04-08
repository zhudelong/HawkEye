#include "dji_bumper/dji_bumper.h"

DJI_lock    g_lock;
DJI_event   g_event;

bool img_arrival[5][2];
cv::Mat grey[5][2];
cv::Mat disp[5];
cv::Mat depth[5];
cv::Mat rescaled_depth[5];
cv::Mat segments[5];

cv::gpu::GpuMat d_grey[5][2];
cv::gpu::GpuMat d_disp[5];
cv::gpu::GpuMat d_depth[5];
cv::gpu::GpuMat d_rescaled_depth[5];
cv::gpu::GpuMat d_segments[5];

obstacle_distance oa;
ultrasonic_data ultrasonic;
std::queue<motion> pose;
std::queue<velocity> vo;
imu imu_data;
stereo_cali cali[5];

unsigned char g_warning_flag;
dji_bumper::DetectionResult dr_msg;

unsigned char g_mode = 1;

#define RETURN_IF_ERR(err_code) { if( err_code ){ release_transfer(); \
std::cout<<"Error: "<<(e_sdk_err_code)err_code<<" at "<<__LINE__<<","<<__FILE__<<std::endl; return -1;}}

/**
 * activated when any data is received
 * @param data_type - name of data type
 * @param data_len - length of data package
 * @param content - content of data package
 */
int djiBumper::my_callback(int data_type, int data_len, char* content)
{
  g_lock.enter();

  /* image */
  if (e_image == data_type && NULL != content)
    {
      image_data* data = (image_data* )content;
//      printf( "frame index:%d,stamp:%d\n", data->frame_index, data->time_stamp );

      for (int i = 0; i != CAMERA_PAIR_NUM; i++)
        {
          if ( data->m_greyscale_image_left[i] )
            {
              grey[i][0] = cv::Mat::zeros(240, 320, CV_8UC1);
              memcpy( grey[i][0].data, data->m_greyscale_image_left[i], 320 * 240 );
              img_arrival[i][0] = true;
#ifdef DISPLAY_GRAY_IMAGE
              cv::imshow(std::string("left") + char(i + '0'), grey[i][0]);
#endif
#ifdef SAVE_LEFT_IMAGE
              static int grey_left_cnt = 0;
              std::ostringstream left_fname;
              left_fname << "/home/" << USER_NAME << "/data_pack/" << DATA_PACK_ID_FOR_SAVING << "/img/" << i << "/grey/left/" << grey_left_cnt << ".jpg";
              cv::imwrite(left_fname.str(), grey[i][0]);
              grey_left_cnt++;
#endif
            }

          if ( data->m_greyscale_image_right[i] )
            {
              grey[i][1] = cv::Mat::zeros(240, 320, CV_8UC1);
              memcpy( grey[i][1].data, data->m_greyscale_image_right[i], 320 * 240 );
              img_arrival[i][1] = true;
#ifdef DISPLAY_GRAY_IMAGE
              cv::imshow(std::string("right") + char(i + '0'), grey[i][1]);
#endif
#ifdef SAVE_RIGHT_IMAGE
              static int grey_right_cnt = 0;
              std::ostringstream right_fname;
              right_fname << "/home/" << USER_NAME << "/data_pack/" << DATA_PACK_ID_FOR_SAVING << "/img/" << i << "/grey/right/" << grey_right_cnt << ".jpg";
              cv::imwrite(right_fname.str(), grey[i][1]);
              grey_right_cnt++;
#endif
            }

          if (data->m_disparity_image[i])
            {
              disp[i] = cv::Mat::zeros(240, 320, CV_16SC1);
              memcpy(disp[i].data, data->m_disparity_image[i], 320 * 240 * 2);
#ifdef DISPLAY_DIPARITY_IMAGE
              static cv::Mat disp_temp = cv::Mat::zeros(240, 320, CV_8UC1);
              disp[i].convertTo(disp_temp, CV_8UC1); // lower 4 bits are fraction
              cv::imshow(std::string("disparity") + char(i + '0'), disp_temp);
#endif
#ifdef SAVE_DISPARITY_IMAGE
              static int disp_cnt = 0;
              std::ostringstream disp_fname;
              disp_fname << "/home/" << USER_NAME << "/data_pack/" << DATA_PACK_ID_FOR_SAVING << "/img/" << i << "/disparity/" << disp_cnt << ".jpg";
              cv::imwrite(disp_fname.str(), disp[i]);
              disp_cnt++;
#endif
            }

          if ( data->m_depth_image[i] )
            {
              depth[i] = cv::Mat::zeros(240, 320, CV_16SC1);
              memcpy(depth[i].data, data->m_depth_image[i], 320 * 240 * 2);
#ifdef DISPLAY_DEPTH_IMAGE
              static cv::Mat depth_temp = cv::Mat::zeros(240, 320, CV_8UC1);
              depth[i].convertTo(depth_temp, CV_8UC1); // lower 7 bits are fraction
              cv::imshow(std::string("depth") + char('0' + i), depth_temp);
#endif
#ifdef SAVE_DEPTH_IMAGE
              static int depth_cnt = 0;
              std::ostringstream depth_fname;
              depth_fname << "/home/" << USER_NAME << "/data_pack/" << DATA_PACK_ID_FOR_SAVING << "/img/" << i << "/depth/" << depth_cnt << ".jpg";
//              std::cout << "saving" << depth_fname.str() << std::endl;
              cv::imwrite(depth_fname.str().c_str(), depth[i]);
              depth_cnt++;
#endif
            }
        }// for
    }// image

  /* obstacle distance */
  if ( e_obstacle_distance == data_type && NULL != content )
    {
      oa = *((obstacle_distance*)content);
#ifdef DISPLAY_OBSTACLE_DISTANCE
      printf( "frame index: %d, stamp: %d\n", oa.frame_index, oa.time_stamp );
      printf( "obstacle distance[m]:" );

      for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
        {
          printf( " %f m", 0.01f * oa.distance[i] );
        }

      printf( "\n" );
#endif
    }

  /* ultrasonic */
  if ( e_ultrasonic == data_type && NULL != content )
    {
      ultrasonic = *((ultrasonic_data*)content);
#ifdef DISPLAY_ULTRASONIC
      printf( "frame index: %d, stamp: %d\n", ultrasonic.frame_index, ultrasonic.time_stamp );

      for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
        {
          printf( "ultrasonic[m]: %f, reliability: %d\n", ultrasonic.ultrasonic[d] * 0.001f, (int)ultrasonic.reliability[d] );
        }

#endif
    }

  if ( e_velocity == data_type && NULL != content )
    {
#ifdef SAVE_VELOCITY

      if (vo.size() < 3)
        {
          vo.push(*((velocity*)content));
        }

      else
        {
          vo.pop();
          vo.push(*((velocity*)content));
        }

      printf( "frame index: %d, stamp: %d\n", vo.back().frame_index, vo.back().time_stamp );
      printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo.back().vx, 0.001f * vo.back().vy, 0.001f * vo.back().vz );
#endif
    }

  /* motion */
  if ( e_motion == data_type && NULL != content )
    {
#ifdef SAVE_MOTION

      if (pose.size() < 3)
        {
//          std::cout << "pose data is not enough" << std::endl;
          pose.push(*((motion*)content));
        }

      else
        {
//          std::cout << "saving motion[" << pose.size() << "/3]:" << std::endl;
          pose.pop();
          pose.push(*((motion*)content));
        }

#endif
    }

  /* imu */
  if ( e_imu == data_type && NULL != content )
    {
#ifdef SAVE_IMU
      imu_data = *((imu*)content);
      printf( "frame index: %d, stamp: %d\n", imu_data.frame_index, imu_data.time_stamp );
      printf( "imu: [%f %f %f %f %f %f %f]\n", imu_data.acc_x, imu_data.acc_y, imu_data.acc_z, imu_data.q[0], imu_data.q[1], imu_data.q[2], imu_data.q[3] );
#endif
    }

  cv::waitKey(1);
  g_lock.leave();
  g_event.set_event();
  return 0;
}

bool djiBumper::switchModeCb(dji_bumper::SwitchMode::Request& req, dji_bumper::SwitchMode::Response& res)
{
  switch(req.mode)
    {
    case 0:
      {
        g_mode = 0;
        break;
      }
    case 1:
      {
        g_mode = 1;
        stop_transfer();
        reset_config();
        select_ultrasonic();
        select_obstacle_distance();
        set_image_frequecy(e_frequecy_10);
        select_greyscale_image(e_vbus3, true);
        select_greyscale_image(e_vbus3, false);
        start_transfer();
        break;
      }
    case 2:
      {
        g_mode = 2;
        stop_transfer();
        reset_config();
        select_ultrasonic();
        select_obstacle_distance();
        set_image_frequecy(e_frequecy_10);
        select_greyscale_image(e_vbus3, true);
        select_depth_image(e_vbus3);
        start_transfer();
        break;
      }
    default:
      {
//        std::cout<<"requesting a unspecified mode!"<<std::endl;
        break;
      }
    }

  res.result = g_mode;
  return 0;
}

/**
 *  constructor of djiBumper
 */
djiBumper::djiBumper()
{
//  std::cout << "construct djiBumper" << std::endl;
  image_width_ = 320;
  image_height_ = 240;
  image_size_ = image_width_ * image_height_;
  max_depth_ = 5.0; // ignore points further than 5.5 meters
  min_depth_ = 0.0; // ignore points closer than 0.5 meter
  path_width_ = 2.0; // garantee width of path is no less than 2.0 meters
  path_height_ = 2.0; // garantee height of path is no less than 2.0 meters
  path_tolerance_ratio_ = 0.001; //
  /* claim ROS topic */
  pub_obj_dis = nh_.advertise<sensor_msgs::LaserScan>("/guidance/obstacle_distance", 1);
  pub_ultrasonic = nh_.advertise<sensor_msgs::LaserScan>("/guidance/ultrasonic", 1);
  pub_detection_result_ = nh_.advertise<dji_bumper::DetectionResult>("/guidance/detection_result", 1);
  pub_imu = nh_.advertise<geometry_msgs::TransformStamped>("/guidance/imu", 1);
  /* service */
  srv_mode = nh_.advertiseService("/guidance/switch_mode",&djiBumper::switchModeCb,this);

  for (int i = 0; i != CAMERA_PAIR_NUM; i++)
    {
      // clear flag
      img_arrival[i][0] = false;
      img_arrival[i][1] = false;
      // claim topic
      // disparity
      pub_img_disp[i] = nh_.advertise<sensor_msgs::Image>(std::string("/guidance/disparity_image/") + char('0' + i), 1);
      // depth
      pub_img_depth[i] = nh_.advertise<sensor_msgs::Image>(std::string("/guidance/depth_image/") + char('0' + i), 1);
    }

#ifdef OFFLINE_DEBUG
  // TODO: this part should be written as a function
  std::ostringstream cali_fname;
  cali_fname<<"/home/"<<USER_NAME<<"/data_pack/cali.yaml";
  cv::FileStorage fs(cali_fname.str(), cv::FileStorage::READ);

  for(int i = 0; i!=CAMERA_PAIR_NUM; i++)
    {
      fs[std::string("cali")+char('0'+i)+"cu"]>>cali[i].cu;
      fs[std::string("cali")+char('0'+i)+"cv"]>>cali[i].cv;
      fs[std::string("cali")+char('0'+i)+"focal"]>>cali[i].focal;
      fs[std::string("cali")+char('0'+i)+"baseline"]>>cali[i].baseline;
    }
  std::cout << "cu\tcv\tfocal\tbaseline\n";

  for (int i = 0; i < CAMERA_PAIR_NUM; i++)
    {
      std::cout << cali[i].cu << "\t" << cali[i].cv << "\t" << cali[i].focal << "\t" << cali[i].baseline << std::endl;
    }

  fs.release();
/*
  cali[0].cu = 164.462;
  cali[0].cv = 122.833;
  cali[0].focal = 243.682;
  cali[0].baseline = 0.150284;
  cali[1].cu = 159.222;
  cali[1].cv = 122.743;
  cali[1].focal = 238.930;
  cali[1].baseline = 0.150034;
  cali[2].cu = 161.137;
  cali[2].cv = 119.936;
  cali[2].focal = 248.683;
  cali[2].baseline = 0.150903;
  cali[3].cu = 156.438;
  cali[3].cv = 119.773;
  cali[3].focal = 254.138;
  cali[3].baseline = 0.150641;
  cali[4].cu = 163.143;
  cali[4].cv = 115.561;
  cali[4].focal = 230.470;
  cali[4].baseline = 0.153080;
  */
#endif
}

/**
 *  destructor of djiBumper
 */
djiBumper::~djiBumper()
{
  std::cout << "destroy djiBumper" << std::endl;
}

/**
 *  initialize djiBumper
 */
int djiBumper::init(void)
{
  std::cout << "initialize djiBumper" << std::endl;
  /* reset */
  reset_config();  // clear all data subscription
  int err_code = init_transfer(); //wait for board ready and init transfer thread
  RETURN_IF_ERR(err_code);
  /* get online status */
  int online_status[CAMERA_PAIR_NUM];
  err_code = get_online_status(online_status);
  RETURN_IF_ERR(err_code);
  std::cout << "[Sensor online status] ";

  for (int i = 0; i < CAMERA_PAIR_NUM; i++)
    std::cout << online_status[i] << " ";

  std::cout << std::endl;
  /* get cali param */
  err_code = get_stereo_cali(cali);
  RETURN_IF_ERR(err_code);
  std::cout << "cu\tcv\tfocal\tbaseline\n";

  for (int i = 0; i < CAMERA_PAIR_NUM; i++)
    {
      std::cout << cali[i].cu << "\t" << cali[i].cv << "\t" << cali[i].focal << "\t" << cali[i].baseline << std::endl;
    }

#ifdef SAVE_CALI
  std::ostringstream cali_fname;
  cali_fname<<"/home/"<<USER_NAME<<"/data_pack/cali.yaml";
  cv::FileStorage fs(cali_fname.str(), cv::FileStorage::WRITE);

  for(int i = 0; i!=CAMERA_PAIR_NUM; i++)
    {
      fs<<std::string("cali")+char('0'+i)+"cu"<<cali[i].cu;
      fs<<std::string("cali")+char('0'+i)+"cv"<<cali[i].cv;
      fs<<std::string("cali")+char('0'+i)+"focal"<<cali[i].focal;
      fs<<std::string("cali")+char('0'+i)+"baseline"<<cali[i].baseline;
    }
#endif

  /* subscribe data */
  select_obstacle_distance();
  select_ultrasonic();
//  select_motion();
//  select_imu();
//  select_velocity();
  set_image_frequecy(e_frequecy_10);
//  select_greyscale_image(e_vbus1, true);
//  select_greyscale_image(e_vbus1, false);
//  select_depth_image(e_vbus2);
  select_greyscale_image(e_vbus3, true);
  select_greyscale_image(e_vbus3, false);
//  select_depth_image(e_vbus4);
  /* start data transfer */
  err_code = set_sdk_event_handler(this->my_callback);
  RETURN_IF_ERR(err_code);
  err_code = start_transfer();
  RETURN_IF_ERR(err_code);
}

/**
 * load image for off-line debug
 * @param id - index of camera pair
 * @param type - image type
 * @param frame - specific frame number
 * @param img_dir - directory of 'img' archive. must end with '/img/'
 */
int djiBumper::load(int id, img_type_t type, int frame, std::string img_dir)
{
//  std::cout<<"load image file in: "<< img_dir <<std::endl;
  g_lock.enter();
  int ret = 0;
  std::ostringstream frame_idx;
  frame_idx << frame;

  switch (type)
    {
    case img_type_grey:
      {
        grey[id][0] = cv::imread(img_dir + char('0' + id) + "/grey/left/" + frame_idx.str() + ".jpg", CV_LOAD_IMAGE_UNCHANGED);
//        std::cout<<"loading: "<<img_dir+char('0'+id) + "/grey/left/" + frame_idx.str() + ".jpg"<<std::endl;
        img_arrival[id][0] = !grey[id][0].empty();
        grey[id][1] = cv::imread(img_dir + char('0' + id)  + "/grey/right/" + frame_idx.str() + ".jpg", CV_LOAD_IMAGE_UNCHANGED);
//        std::cout<<"loading: "<<img_dir+char('0'+id)  + "/grey/right/" + frame_idx.str() + ".jpg"<<std::endl;
        img_arrival[id][1] = !grey[id][1].empty();

        if (img_arrival[id][0] == true && img_arrival[id][1] == true)
          {
//            std::cout<<"received new greyscale image pair"<<std::endl;
            ret = 0;
#ifdef DISPLAY_LOADED_IMAGE
            cv::imshow(std::string("loaded_left") + char('0' + id), grey[id][0]);
            cv::imshow(std::string("loaded_right") + char('0' + id), grey[id][1]);
#endif
          }

        break;
      }

    case img_type_disparity:
      {
        disp[id] = cv::imread(img_dir + char('0' + id) + "/disparity/" + char('0' + frame) + ".jpg", CV_LOAD_IMAGE_UNCHANGED);
        ret = disp[id].empty();

        if (ret == 0 )
          {
#ifdef DISPLAY_LOADED_IMAGE
            cv::imshow(std::string("loaded_disparity") + char('0' + id), disp[id]);
#endif
          }

        break;
      }

    case img_type_depth:
      {
        depth[id] = cv::imread(img_dir + char('0' + id) + "/depth/" + char('0' + frame) + ".jpg", CV_LOAD_IMAGE_UNCHANGED);
        ret = depth[id].empty();

        if (ret == 0 )
          {
#ifdef DISPLAY_LOADED_IMAGE
            cv::imshow(std::string("loaded_depth") + char('0' + id), depth[id]);
#endif
          }

        break;
      }

    default:
      {
//        std::cout << "parameter error occurs when trying to load image!" << std::endl;
        ret = 1;
        break;
      }
    }

  cv::waitKey(1);
  g_lock.leave();
  return ret;
}


/**
 * stereo match
 * @param id - specify id of camera pair to be matched
 * @param ndisparities - default value 64
 * @param winSize - default value 19
 * @param match_mode - the method used to conduct stereo match
 * @return none-zero value means error
 */
int djiBumper::match(int id, int ndisparities, int winSize, match_mode_t mode)
{
  g_lock.enter();
  static int ret_val = 0;

  if (id <= 4 && id >= 0)
    {
      cv::gpu::StereoBM_GPU bm(CV_STEREO_BM_BASIC, ndisparities, winSize);
      cv::gpu::StereoBeliefPropagation bp(ndisparities);
      cv::gpu::StereoConstantSpaceBP csbp(ndisparities);
      cv::StereoBM bm_cpu(CV_STEREO_BM_BASIC, ndisparities, winSize);
      cv::StereoSGBM sgbm_cpu(0, ndisparities, winSize);

      if (img_arrival[id][0] == true && img_arrival[id][1] == true)
        {
//          std::cout << "match pair[" << id << "] with method ";
        cv::medianBlur(grey[id][0], grey[id][0], 3);
//          cv::GaussianBlur(grey[id][0], grey[id][0], cv::Size(3, 3), 1);
          d_grey[id][0].upload(grey[id][0]);
          img_arrival[id][0] = false;
        cv::medianBlur(grey[id][0], grey[id][0], 3);
//          cv::GaussianBlur(grey[id][0], grey[id][0], cv::Size(5, 5), 1);
          d_grey[id][1].upload(grey[id][1]);
          img_arrival[id][1] = false;

          switch (mode)
            {
            case match_mode_BM:
              {
//                std::cout << "BM" << std::endl;
                bm(d_grey[id][0], d_grey[id][1], d_disp[id]);
                d_disp[id].download(disp[id]);
                break;
              }

            case match_mode_BP:
              {
//                std::cout << "BP" << std::endl;
                bp(d_grey[id][0], d_grey[id][1], d_disp[id]);
                d_disp[id].download(disp[id]);
                break;
              }

            case match_mode_CSBP:
              {
//                std::cout << "CSBP" << std::endl;
                csbp(d_grey[id][0], d_grey[id][1], d_disp[id]);
                d_disp[id].download(disp[id]);
                break;
              }

            case match_mode_BM_CPU:
              {
//                std::cout << "BM_CPU" << std::endl;
                bm_cpu(grey[id][0], grey[id][1], disp[id], CV_16S);
                break;
              }

            case match_mode_SGBM_CPU:
              {
//                std::cout << "SGBM_CPU" << std::endl;
                sgbm_cpu(grey[id][0], grey[id][1], disp[id]);
                break;
              }

            default:
              {
//                std::cout << "Unspecified Matching Method!" << std::endl;
                ret_val = 1;
                break;
              }
            }

#ifdef DISPLAY_MATCH_RESULT
          cv::Mat temp_disp_view;
          disp[id].convertTo(temp_disp_view, CV_8UC1);
          cv::imshow(std::string("disp_diy") + char(id + '0'), temp_disp_view);
          cv::waitKey(1);
#endif
#ifdef SAVE_MATCH_RESULT
          static disp_diy_cnt = 0;
          std::ostringstream disp_diy_fname;
          disp_diy_fname << "/home/" << USER_NAME << "/data_pack/" << DATA_PACK_ID_FOR_SAVING << "/img/disp_diy/" << i << "/" << cnt << ".jpg";
          cv::imwrite(disp_diy_fname, disp[i]);
          disp_diy_fname++;
#endif
        }

      else
        {
//          std::cout << "not ready for match" << std::endl;
          ret_val = 1;
        }
    }

  g_lock.leave();
  return ret_val;
}

/**
 * recover depth from disparity
 */
int djiBumper::triangulate(int id)
{
  g_lock.enter();

  if (id <= 4 && id >= 0)
    {
      if (disp[id].empty() == false)
        {
          float coeffcient = cali[id].focal * cali[id].baseline * 16.0 * 128.0; // 128 is for consistency with official data format
          d_disp[id].upload(disp[id]);
          cv::gpu::GpuMat depth_raw;
          cv::gpu::divide(coeffcient, d_disp[id], depth_raw);
          depth_raw.download(depth[id]);
#ifdef DISPLAY_TRIANULATION_RESULT
//      std::cout<<"depth["<<id<<"] is empty:"<<depth[id].empty()<<std::endl;
          cv::Mat rescaled;
          depth[id].convertTo(rescaled, CV_8UC1);
          //cv::convertScaleAbs(depth[id], rescaled, 51/128.0);
          cv::imshow(std::string("depth_diy") + char(id + '0'), rescaled);
          cv::waitKey(1);
#endif
#ifdef SAVE_TRIANGULATION_RESULT
          static depth_diy_cnt = 0;
          std::ostringstream depth_diy_fname;
          depth_diy_fname << "/home/" << USER_NAME << "/data_pack/" << DATA_PACK_ID_FOR_SAVING << "/img/depth_diy/" << i << "/" << cnt << ".jpg";
          cv::imwrite(depth_diy_fname, depth[i]);
          depth_diy_fname++;
#endif
        }
    }

  g_lock.leave();
  return 0;
}

/**
 * generate point cloud from depth image

int djiBumper::generateCloud(int id)
{

}
 */

/**
 * segment depth image into pieces
 */
int djiBumper::segment(int id)
{
  g_lock.enter();
//  std::cout << "djiBumper::segment() is called" << std::endl;

  if (id <= 4 && id >= 0)
    {
      if (depth[id].empty() == false)
        {
//          std::cout << "segment depth[" << id << "]" << std::endl;
          d_depth[id].upload(depth[id]);
          // rescale the image
          cv::gpu::GpuMat d_byte_mat;
          d_depth[id].convertTo(d_byte_mat, CV_8UC1, 1 / 2.5);
          // calculate the histgram
          cv::gpu::GpuMat d_hist;
          cv::gpu::calcHist(d_byte_mat, d_hist);
          // threshold
          cv::gpu::GpuMat thresholded;
          cv::gpu::threshold(d_hist, thresholded, 1499, 1499, CV_THRESH_TRUNC);
          // normalize the image
          //cv::gpu::GpuMat normalized_hist;
          cv::Mat graph = cv::Mat::zeros(cv::Size(3 * 256, 500), CV_8UC1);
          cv::Mat tmp_mat;
          thresholded.download(tmp_mat);

          for (int i = 0; i != d_hist.cols; i++)
            {
//              std::cout<<"["<<i<<"] "<<tmp_mat.at<int>(i)<<"\t";
//              if(i%4 == 0)
//                {
//                  std::cout<<std::endl;
//                }
              //cv::circle(graph, cv::Point(i * 3 + 1, 500 - (tmp_mat.at<int>(i) / 3)), 3, cv::Scalar(255, 255, 255));
              cv::line(graph, cv::Point(i * 3 + 1, 499), cv::Point(i * 3 + 1, 500 - (tmp_mat.at<int>(i) / 3)), cv::Scalar(255, 255, 255), 3);
            }

//          cv::Mat byte_mat;
//          d_byte_mat.download(byte_mat);
          cv::imshow(std::string("seg") + char(id + '0'), graph);
          cv::waitKey(1);
#ifdef SAVE_HIST
          static int hist_cnt = 0;
          std::ostringstream hist_fname;
          hist_fname << "/home/" << USER_NAME << "/data_pack/" << DATA_PACK_ID_FOR_SAVING << "/img/" << id << "/hist/" << hist_cnt << ".jpg";
          cv::imwrite(hist_fname.str(), graph);
          hist_cnt++;
#endif
        }
    }

  g_lock.leave();
  return 0;
}



/**
 * check the window
 * @param win - the window to be checked
 * @param sum - total amount of obstacle points
 * @param (vx, vy) - position of obstacle distribution center
 */
void checkWindow(cv::Mat& win, int&sum, int& vx, int& vy)
{
  int temp_sum = 0;
  for(int i = 0; i != win.rows;i++)
    {
      for(int j = 0; j != win.cols;j++)
        {
          if(win.at<unsigned char>(cv::Point(j,i)) != 0)
            {
              temp_sum++;
              vx += (j-(win.cols/2)); // right is positive direction
              vy += (i-(win.rows/2)); // down is positive direction
            }
        }
    }
  sum = temp_sum;
}

/**
 * check a single layer by trying to fit in a rectangle representing possible path
 * @param layer - the layer under examination
 * @param path_width - width of path. unit is pixel
 * @param path_height - height of path. unit is pixel
 * @param pt_thres - threshold of points indicating exsistence of obstacle
 * @param judge - judgement of current layer
 * @param offset_x - x part of a vector, which is from image center to center of path
 * @param offset_y - y part of a vector, which is from image center to center of path
 */
int djiBumper::checkLayer(cv::Mat& layer, int& window_width, int& window_height, int& pt_thres, layer_prop_t& judge, float& offset_x, float& offset_y)
{
  /* get rid of noise */
  // medianBlur
  static cv::Mat filtered;
  cv::medianBlur(layer, filtered,3);
  // erosion
  static cv::Mat eroded;
  cv::Mat kernel_mat = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(filtered, eroded, cv::MORPH_ERODE, kernel_mat);

  /* test the possbility for direct going through */
  cv::Mat window(filtered,cv::Rect((layer.cols-window_width)/2,(layer.rows-window_height)/2, window_width, window_height));
  // count points and calculate the mass center
  int sum = 0;
  int v_x = 0;
  int v_y = 0;
  checkWindow(window,sum,v_x,v_y);

  if(sum < pt_thres) // TODO: find a better priciple for accepting a window
    {
      judge = layer_prop_safe;
      offset_x = offset_y = 0;  
      return 0;
    }

  /* attemp to move */
  // cross searching
  bool found_path = false;
  cv::Mat up(filtered,cv::Rect((layer.cols - window_width)/2,(layer.rows - window_height)/2,window_width,window_height));
  cv::Mat down(filtered,cv::Rect((layer.cols - window_width)/2,(layer.rows - window_height)/2,window_width,window_height));
  cv::Mat left(filtered,cv::Rect((layer.cols - window_width)/2,(layer.rows - window_height)/2,window_width,window_height));
  cv::Mat right(filtered,cv::Rect((layer.cols - window_width)/2,(layer.rows - window_height)/2,window_width,window_height));
  for(int i = 0; i!= (layer.rows-window_height)/2; i++)
    {
      int temp_sum;
      int temp_vx;
      int temp_vy;
      // up
      up.adjustROI(1,-1,0,0);
      checkWindow(up,temp_sum, temp_vx, temp_vy);
      if(temp_sum < sum)
        {
          sum = temp_sum;
          offset_x = 0;
          offset_y = -i;
          if(sum < pt_thres) // TODO: find a better priciple for accepting a window
            {
              judge = layer_prop_avoidable;
              found_path = true;
              break;
            }
        }
      //down
      down.adjustROI(-1,1,0,0);
      checkWindow(down,temp_sum, temp_vx, temp_vy);
      if(temp_sum < sum)
        {
          sum = temp_sum;
          offset_x = 0;
          offset_y = i;
          if(sum < pt_thres) // TODO: find a better priciple for accepting a window
            {
              judge = layer_prop_avoidable;
              found_path = true;
              break;
            }
        }
      //left
      left.adjustROI(0,0,1,-1);
      checkWindow(left,temp_sum, temp_vx, temp_vy);
      if(temp_sum < sum)
        {
          sum = temp_sum;
          offset_x = -i;
          offset_y = 0;
          if(sum < pt_thres) // TODO: find a better priciple for accepting a window
            {
              judge = layer_prop_avoidable;
              found_path = true;
              break;
            }
        }
      //right
      right.adjustROI(0,0,-1,1);
      checkWindow(right,temp_sum, temp_vx, temp_vy);
      if(temp_sum < sum)
        {
          sum = temp_sum;
          offset_x = i;
          offset_y = 0;
          if(sum < pt_thres) // TODO: find a better priciple for accepting a window
            {
              judge = layer_prop_avoidable;
              found_path = true;
              break;
            }
        }
    }

  if(found_path == true)
    {
      return 1;
    }
  else
    {
      judge = layer_prop_blocked;
      return 2;
    }
}

/**
 * pack ROS message
 */
void packDetectionResult(int& id, float& dist, float& offs_x, float& offs_y, layer_prop_t& sta,dji_bumper::DetectionResult& msg)
{

  unsigned char status;
  switch(sta)
    {
     case layer_prop_safe:
      {
        status = 0;
        break;
      }
    case layer_prop_avoidable:
     {
       status = 1;
       break;
     }
    case layer_prop_blocked:
     {
       status = 2;
       break;
     }
    default:
     {
//       std::cout<<"wrong status"<<std::endl;
       break;
     }
    }


  switch(id)
    {
    case 0:
      {
        msg.window_pos_down.x = dist + 0.1;
        msg.window_pos_down.y = offs_x * dist/cali[id].focal;
        msg.window_pos_down.z = 0.09 - (offs_y * dist/cali[id].focal);
        msg.window_sta_down = status;
        break;
      }
    case 1:
      {
        msg.window_pos_front.x = dist + 0.1;
        msg.window_pos_front.y = offs_x * dist/cali[id].focal;
        msg.window_pos_front.z = 0.09 - (offs_y * dist/cali[id].focal);
        msg.window_sta_front =status;
        break;
      }
    case 2:
      {
        msg.window_pos_right.x = -offs_x * dist/cali[id].focal;
        msg.window_pos_right.y = dist + 0.1;
        msg.window_pos_right.z = 0.09 - (offs_y * dist/cali[id].focal);
        msg.window_sta_right = status;
        break;
      }
    case 3:
      {
        msg.window_pos_rear.x = -dist - 0.1;
        msg.window_pos_rear.y = -offs_x * dist/cali[id].focal;
        msg.window_pos_rear.z = 0.09 - (offs_y * dist/cali[id].focal);
        msg.window_sta_rear = status;
        break;
      }
    case 4:
      {
        msg.window_pos_left.x = offs_x * dist/cali[id].focal;
        msg.window_pos_left.y = -dist - 0.1;
        msg.window_pos_left.z = 0.09 - (offs_y * dist/cali[id].focal);
        msg.window_sta_left = status;
        break;
      }
    default:
      {
//        std::cout<<"wrong id"<<std::endl;
        break;
      }
    }

}

/**
 * detect if there is a way out
 */
int djiBumper::detectPath(int id, int layer_num, float& safe_dist, float& offset_x, float& offset_y)
{
  g_lock.enter();
//  std::cout << "detect Path in direction[" << id << "]" << std::endl;

  if (depth[id].empty() == false)
    {
      /* slice depth image into several layers */
      static float layer_thickness = (max_depth_ - min_depth_) / layer_num; // this value only needs to be initialized once, unit is meter
      std::vector<cv::Mat> layers;

      for (int i = 0; i != layer_num; i++)
        {
          d_depth[id].upload(depth[id]);
          // get rid of points that are too close
          static cv::gpu::GpuMat no_closer_pt;
          cv::gpu::threshold(d_depth[id], no_closer_pt, i * layer_thickness * 128.0, 0, CV_THRESH_TOZERO);
          // get rid of points that are too far
          static cv::gpu::GpuMat no_further_pt;
          cv::gpu::threshold(no_closer_pt, no_further_pt, (i + 1)*layer_thickness * 128.0, 0, CV_THRESH_TOZERO_INV);
          // convert into binary image
          static cv::gpu::GpuMat binary;
          cv::gpu::threshold(no_further_pt,binary,i * layer_thickness * 128.0, 255, CV_THRESH_BINARY);
          // convert to byte image
          static cv::gpu::GpuMat byte_img;
          binary.convertTo(byte_img,CV_8UC1);
          // save layer
          static cv::Mat layer;
          byte_img.download(layer);
          layers.push_back(layer);
#ifdef DISPLAY_LAYER
          cv::imshow(std::string("pair") + char('0' + id) + "layer" + char('0' + i), layer);
          cv::waitKey(1);
#endif
        }

      /* examin each layer */
      static int roi_width, roi_height;
      layer_prop_t judgement;
      for (int i = 1; i != layer_num; i++)
        {
//          std::cout << "check layer [" << i << "]" << std::endl;

          // calculate window size in current layer
          roi_width = path_width_ * cali[id].focal  / (i * layer_thickness);

          if (roi_width > image_width_) roi_width = image_width_;

          roi_height = path_height_ * cali[id].focal  / (i * layer_thickness);

          if (roi_height > image_height_) roi_height = image_height_;

//          std::cout << "window size: " << roi_width << " * " << roi_height << std::endl;

          // try to fit the window in current layer
          int point_threshold = roi_width * roi_height * path_tolerance_ratio_; // TODO: calculate threshold based on depth!!!!!!!!!!!!!!
//          std::cout<<"threshold:"<<point_threshold<<std::endl;
          checkLayer(layers[i],roi_width,roi_height,point_threshold,judgement,offset_x,offset_y);

          bool terminate_flag = false;
          switch(judgement)
            {
            case layer_prop_safe:
              {
                safe_dist = (i+1)*layer_thickness; // save current safe distance and continue to check next layer
                break;
              }
            case layer_prop_avoidable:
              {
                safe_dist = i*layer_thickness;
                terminate_flag = true;
                break;
              }
            case layer_prop_blocked:
              {
                safe_dist = i*layer_thickness;
                terminate_flag = true;
                break;
              }
            default:
              {
//                std::cout<<"unspecified layer property!"<<std::endl;
                terminate_flag = true;
                break;
              }

            }
          if(terminate_flag == true)
            {
              break;
            }

        }

      /* pack data as ROS message */
      packDetectionResult(id,safe_dist,offset_x,offset_y,judgement,dr_msg);

#ifdef DISPLAY_PATH
      // be colorful
      static cv::Mat colorful;
      cv::cvtColor(grey[id][0],colorful,CV_GRAY2BGR);
      // choose color
      cv::Scalar sca;
      if(safe_dist >= max_depth_)
        {
          sca=cv::Scalar(0,255,0);
        }
      else if(safe_dist > layer_thickness)
        {
          sca=cv::Scalar(0,255,255);
        }
      else
        {
          sca=cv::Scalar(0,0,255);
        }
      // draw window
      cv::Rect rect_view(((image_width_-roi_width)/2)+offset_x,((image_height_-roi_height)/2)+offset_y, roi_width, roi_height);
      cv::rectangle(colorful,rect_view,sca);
      // draw offset
      cv::arrowedLine(colorful,cv::Point(colorful.cols/2,colorful.rows/2),cv::Point(colorful.cols/2+offset_x,colorful.rows/2+offset_y),sca,3);
      // put text(distance)
      std::ostringstream dis_val;
      dis_val<<"Safety Diatance: "<<safe_dist<<" m";
      cv::putText(colorful,dis_val.str(),cv::Point(0,colorful.rows - 2), cv::FONT_HERSHEY_PLAIN,0.8,sca);
      // put text(mode)
      switch(g_mode)
        {
        case 1:
          {
            cv::putText(colorful,"Customized",cv::Point(240,colorful.rows - 2), cv::FONT_HERSHEY_PLAIN,0.8,sca);
            break;
          }
        case 2:
          {
            cv::putText(colorful,"Default   ",cv::Point(240,colorful.rows - 2), cv::FONT_HERSHEY_PLAIN,0.8,sca);
            break;
          }
        default:
          {
            cv::putText(colorful,"Wrong Mode",cv::Point(240,colorful.rows - 2), cv::FONT_HERSHEY_PLAIN,0.8,sca);
            break;
          }
        }
      // draw signal light

      if(safe_dist >= max_depth_)
        {
          cv::circle(colorful,cv::Point(5,5),5,cv::Scalar(0,255,0),CV_FILLED);
        }
      else if(safe_dist > layer_thickness)
        {
          cv::circle(colorful,cv::Point(15,5),5,cv::Scalar(0,255,255),CV_FILLED);
        }
      else
        {
          cv::circle(colorful,cv::Point(25,5),5,cv::Scalar(0,0,255),CV_FILLED);
        }


      // display
      cv::imshow(std::string("path")+char('0'+id),colorful);
      cv::waitKey(1);
#endif
    }

  else
    {
//      std::cout << "no depth information for path detection" << std::endl;
    }

  g_lock.leave();
  return 0;
}

/**
 * publish ros message
 */
void djiBumper::publish(void)
{
  g_lock.enter();
#ifdef PUBLISH_ULTRASONIC
  /* ultrasonic */
  static sensor_msgs::LaserScan g_ul;
  g_ul.ranges.resize(CAMERA_PAIR_NUM);
  g_ul.intensities.resize(CAMERA_PAIR_NUM);
  g_ul.header.frame_id = "guidance";
  g_ul.header.stamp    = ros::Time::now();

  for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
    {
      g_ul.ranges[d] = 0.001f * ultrasonic.ultrasonic[d];
      g_ul.intensities[d] = 1.0 * ultrasonic.reliability[d];
    }

  pub_ultrasonic.publish(g_ul);
#endif
#ifdef PUBLISH_OBSTACLE_DISTANCE
  /* obstacle distance */
  static sensor_msgs::LaserScan g_oa;
  g_oa.ranges.resize(CAMERA_PAIR_NUM);
  g_oa.header.frame_id = "guidance";
  g_oa.header.stamp    = ros::Time::now();

  for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
    g_oa.ranges[i] = 0.01f * oa.distance[i];

  pub_obj_dis.publish(g_oa);
#endif

  /* image data */
  for (int i = 0; i != CAMERA_PAIR_NUM; i++)
    {
#ifdef PUBLISH_DISPARITY_IMAGE
      // disparity
      static cv_bridge::CvImage cv_img_disp[5];

      if (disp[i].empty() == false)
        {
          cv_img_disp[i].header.stamp = ros::Time::now();
          std::ostringstream frame_str;
          frame_str << "pair" << i;
          cv_img_disp[i].header.frame_id = frame_str.str();
          cv_img_disp[i].encoding = sensor_msgs::image_encodings::MONO16;
          cv_img_disp[i].image = disp[i];
          pub_img_disp[i].publish(cv_img_disp[i].toImageMsg());
        }

#endif
#ifdef PUBLISH_DEPTH_IMAGE
      // depth
      static cv_bridge::CvImage cv_img_depth[5];

      if (depth[i].empty() == false)
        {
          cv_img_depth[i].header.stamp	  = ros::Time::now();
          std::ostringstream frame_str;
          frame_str << "pair" << i;
          cv_img_depth[i].header.frame_id  = frame_str.str();
          cv_img_depth[i].encoding = sensor_msgs::image_encodings::MONO16;
          cv_img_depth[i].image = depth[i];
          pub_img_depth[i].publish(cv_img_depth[i].toImageMsg());
        }

#endif
    }

#ifdef PUBLISH_MOTION
  // motion
//  std::cout << "publish tf" << std::endl;
  static tf2_ros::TransformBroadcaster br;
  static geometry_msgs::TransformStamped tf_guidance;

  if (pose.size() == 3)
    {
      tf_guidance.header.stamp = ros::Time::now();
      tf_guidance.header.frame_id = "world";
      tf_guidance.child_frame_id = "guidance";
      tf_guidance.transform.translation.x = pose.back().position_in_global_x;
      tf_guidance.transform.translation.y = pose.back().position_in_global_y;
      tf_guidance.transform.translation.z = pose.back().position_in_global_z;
      tf_guidance.transform.rotation.x = pose.back().q0;
      tf_guidance.transform.rotation.y = pose.back().q1;
      tf_guidance.transform.rotation.z = pose.back().q2;
      tf_guidance.transform.rotation.w = pose.back().q3;
      br.sendTransform(tf_guidance);
    }

#endif
#ifdef PUBLISH_IMU
  static geometry_msgs::TransformStamped g_imu;
  g_imu.header.frame_id = "guidance";
  g_imu.header.stamp    = ros::Time::now();
  g_imu.transform.translation.x = imu_data.acc_x;
  g_imu.transform.translation.y = imu_data.acc_y;
  g_imu.transform.translation.z = imu_data.acc_z;
  g_imu.transform.rotation.w = imu_data.q[0];
  g_imu.transform.rotation.x = imu_data.q[1];
  g_imu.transform.rotation.y = imu_data.q[2];
  g_imu.transform.rotation.z = imu_data.q[3];
  pub_imu.publish(g_imu);
#endif
#ifdef PUBLISH_VELOCITY
  static geometry_msgs::Vector3Stamped g_vo;
  g_vo.header.frame_id = "guidance";
  g_vo.header.stamp    = ros::Time::now();
  g_vo.vector.x = 0.001f * vo.back().vx;
  g_vo.vector.y = 0.001f * vo.back().vy;
  g_vo.vector.z = 0.001f * vo.back().vz;
  pub_vel.publish(g_vo);
#endif
#ifdef PUBLISH_DETECTION_RESULT

//  static dji_bumper::ObstacleStatus g_os;
//  g_os.header.frame_id = "guidance";
//  g_os.header.stamp = ros::Time::now();
//  g_os.warning_flag = g_warning_flag;
//  pub_obstacle_status_.publish(g_os);
  dr_msg.header.stamp = ros::Time::now();
  dr_msg.header.frame_id = "guidance";
  dr_msg.warning_flag = g_warning_flag;
  pub_detection_result_.publish(dr_msg);

#endif
g_lock.leave();
}



/**
 * check safety by vision
 * @param threshold_h - max distance of dangerous range(meters)
 * @param threshold_l - min distance of dangerous range(meters)
 */
unsigned char djiBumper::checkSafetyByVision(float threshold_h, float threshold_l, int threshold_cnt)
{
  g_lock.enter();
  static unsigned char result_history[2];
  static unsigned char result_temp;
  result_temp = 0;

  for (int i = 0; i != CAMERA_PAIR_NUM; i++)
    {
      if (depth[i].empty() == false)
        {
          d_depth[i].upload(depth[i]);
//          cv::gpu::GpuMat smooth;
//          cv::gpu::GaussianBlur(d_depth[i],smooth,cv::Size(5,5),1,1);
          cv::gpu::GpuMat down_sampled;
          cv::gpu::resize(d_depth[i], down_sampled, cv::Size(80, 60));
          cv::gpu::GpuMat not_so_far;
          cv::gpu::threshold(down_sampled, not_so_far, threshold_h * 128.0, 255, CV_THRESH_BINARY_INV);
          cv::gpu::GpuMat not_so_close;
          cv::gpu::threshold(down_sampled, not_so_close, threshold_l * 128.0, 255, CV_THRESH_BINARY);
          cv::gpu::GpuMat in_region;
          cv::gpu::bitwise_and(not_so_far, not_so_close, in_region);
          cv::Mat tmp1, tmp2;
          in_region.download(tmp1);
          tmp1.convertTo(tmp2, CV_8UC1);
//          cv::imshow(std::string("in_region") + char(i + '0'), tmp2);
//          cv::waitKey(1);
          int pt_cnt = 0;
          bool found_obstacle = false;

          // check number of points in detection region
          for (int i = 0; i != 30; i++)
            {
              // vertical
              if (tmp2.at<unsigned char>(30 + i, 40) != 0)
                {
                  pt_cnt++;
                }

              if (tmp2.at<unsigned char>(30 - i, 40) != 0)
                {
                  pt_cnt++;
                }

              // horisontal
              if (tmp2.at<unsigned char>(30, 40 + i) != 0)
                {
                  pt_cnt++;
                }

              if (tmp2.at<unsigned char>(30, 40 - i) != 0)
                {
                  pt_cnt++;
                }

              // incline
              if (tmp2.at<unsigned char>(30 + i, 40 + i) != 0)
                {
                  pt_cnt++;
                }

              if (tmp2.at<unsigned char>(30 + i, 40 - i) != 0)
                {
                  pt_cnt++;
                }

              if (tmp2.at<unsigned char>(30 - i, 40 + i) != 0)
                {
                  pt_cnt++;
                }

              if (tmp2.at<unsigned char>(30 - i, 40 - i) != 0)
                {
                  pt_cnt++;
                }

              // count pixel
              if (pt_cnt > threshold_cnt)
                {
                  found_obstacle = true;
                  //break;
                }
            }

//          std::cout << "direction[" << i << "]dangerous point:" << pt_cnt << std::endl;

          if (found_obstacle == true)
            {
              result_temp |= (1 << i);
            }
        }
    }

  result_history[0] = result_history[1];
  result_history[1] = result_temp;
  g_lock.leave();
  return (result_history[0] & result_history[1]);
}

/**
 * check safety by ultrasonic
 * @param threshold_h - max distance of dangerous range(meters)
 * @param threshold_l - min distance of dangerous range(meters)
 */
unsigned char djiBumper::checkSafetyByUltrasonic(float threshold_h, float threshold_l)
{
  g_lock.enter();
  static unsigned char result_history[2];
  static unsigned char result_temp;
  result_temp = 0;

  for (int i = 0; i != CAMERA_PAIR_NUM; i++)
    {
      if (ultrasonic.reliability[i] != 0)
        {
          if (ultrasonic.ultrasonic[i] < threshold_h && ultrasonic.ultrasonic[i] > threshold_l)
            {
              result_temp |= (1 << i);
            }
        }
    }

  result_history[0] = result_history[1];
  result_history[1] = result_temp;
  g_lock.leave();
  return (result_history[0] & result_history[1]);
}
