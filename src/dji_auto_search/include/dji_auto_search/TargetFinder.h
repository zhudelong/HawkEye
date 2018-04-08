#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
//#define SHOW_RESULT
using namespace cv;
using namespace std;

class TargetFinder
{
public:
    TargetFinder();

private:
    int bi_sig;
    int min_area;
    int sp ;
    int sr;
    int sobel_ker_size ;
    int median_ker_size;

public:
    bool set_gpu_device(int dev = 0);
    float get_median(Mat& channel) const;
    vector<Mat> get_sobel_channels(Mat& ori);
    void get_canny_channels(Mat& ori,vector<Mat>&edges);
    vector<RotatedRect> get_candidate_areas(Mat& ori_gray);
    vector<RotatedRect> get_selected_rects(vector<Mat>& edges);


};

//int main(int, char**)
//{
//    TargetFinder* finder = new TargetFinder();
//    finder->set_gpu_device(0);
//    Mat ori = imread("/home/zhudelong/img/other/c12.jpg");
//    if(ori.channels() < 3){
//        std::cout<<"too less channels"<<std::endl;
//        return -1;
//    }
//    finder->get_candidate_areas(ori);
//    vector<Mat> edges = finder->get_canny_channels(ori);
//    finder->get_selected_rects(edges);
//    waitKey(0);
//    if(waitKey(30) >= 0)
//        return 1;

//    delete finder;
//}
