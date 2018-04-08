1.标定
------

rosdep install camera_calibration
rosmake camera_calibration

roslaunch video_stream_opencv calibration.launch （注意其中的视频流来源及棋盘格大小）

在/tmp下生成calibrationdata.tar.gz文件，解压ost.txt至video_stream_opencv/config下

cd至config
mv ost.txt ost.ini
rosrun  camera_calibration_parsers convert  ost.ini camera.yml


2.运行
------

roslaunch apriltags apriltags.launch


**注：**第一次编译需要联网,并将apriltags_ros_new/CMakeLists.txt#55取消注释. 注意更改launch文件中的视频流来源、tag的id及tag的大小，id见apriltags_ros/tags中的对应pdf，大小以最外黑边框为准
