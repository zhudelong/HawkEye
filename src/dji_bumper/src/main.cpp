#include <ros/ros.h>
#include <iostream>
#include "dji_bumper/dji_bumper.h"

extern DJI_event   g_event;   // activated every time callback is called
extern unsigned char g_warning_flag;  // corresponding bit would be set once any obstacle found in detection region
extern unsigned char g_mode;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dji_bumper");
  djiBumper bumper; // create a instance of class djiBumper
#ifndef OFFLINE_DEBUG

  if (bumper.init() != 0 ) // initialize bumper
    {
      std::cout << "failed to initialize guidance" << std::endl;
      return 1;
    }

#endif

  while (ros::ok())
    {
#ifndef OFFLINE_DEBUG
      g_event.wait_event();
#else
      const int img_seq_start = 0;
      const int img_seq_end = 550;
      static int img_idx = img_seq_start;
//      int load_flag_1 = bumper.load(1, img_type_grey, img_idx, std::string("/home/") + USER_NAME + "/data_pack/" + char('0' + DATA_PACK_ID_FOR_LOADING) + "/img/"); // switch pack number if necessary
//      int load_flag_2 = bumper.load(2, img_type_grey, img_idx, std::string("/home/") + USER_NAME + "/data_pack/" + char('0' + DATA_PACK_ID_FOR_LOADING) + "/img/"); // switch pack number if necessary
      int load_flag_3 = bumper.load(3, img_type_grey, img_idx, std::string("/home/") + USER_NAME + "/data_pack/" + char('0' + DATA_PACK_ID_FOR_LOADING) + "/img/"); // switch pack number if necessary
//      int load_flag_4 = bumper.load(4, img_type_grey, img_idx, std::string("/home/") + USER_NAME + "/data_pack/" + char('0' + DATA_PACK_ID_FOR_LOADING) + "/img/"); // switch pack number if necessary

      if ((++img_idx) == img_seq_end) img_idx = img_seq_start;

      if (load_flag_3 == 0)
        {
#endif
      switch(g_mode)
        {
        case 0:
          {
//            std::cout<<"mode 0:off mode"<<std::endl;
            break;
          }
        case 1:
          {
//            std::cout<<"mode 1:customized mode"<<std::endl;
            /* get disparity */
            bumper.match(3, 64, 11, match_mode_BM_CPU);
            /* get depth */
            bumper.triangulate(3);
            /* segment image */
      //      bumper.segment(3);
            /* detect path*/
            float safe_dis3, off_x3, off_y3;
            bumper.detectPath(3, 10, safe_dis3, off_x3, off_y3);
            /* check safety
            unsigned char safe_vision = bumper.checkSafetyByVision();
            unsigned char safe_ultrasonic = bumper.checkSafetyByUltrasonic();
            g_warning_flag = safe_vision | safe_ultrasonic;*/
            /* publish ROS message*/

            /* display warning flag (for debug only)
            unsigned char temp_flag = g_warning_flag;
            std::ostringstream tmp_str;

            for (int i = 0; i != 8; i++)
              {
                tmp_str << "[" << i << "]-" << char((temp_flag & 0x01) + '0') << "  ";
                temp_flag >>= 1;
              }

            std::cout << "warning flag:" << tmp_str.str() << std::endl;*/

            break;
          }
        case 2:
          {
//            std::cout<<"mode 2: OEM mode"<<std::endl;

            float safe_dis3, off_x3, off_y3;
            bumper.detectPath(3, 10, safe_dis3, off_x3, off_y3);
            bumper.publish();

            break;
          }
        default:
          {
            std::cout<<"unspecified mode"<<std::endl;
            break;
          }
        }

#ifdef OFFLINE_DEBUG
    }

#endif
  bumper.publish();
  ros::spinOnce();
}

/* release data transfer */
stop_transfer();
sleep(1);
std::cout << "release_transfer" << std::endl;
release_transfer();
return 0;
}
