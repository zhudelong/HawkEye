#ifndef DJI_SCISSOR_H
#define DJI_SCISSOR_H

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>

class djiScissor{
private:
  cv::Mat watershedSegment(cv::Mat & image, int & noOfSegments);
  void mergeSegments(cv::Mat & image, cv::Mat & segments, int & numOfSegments);
  cv::Mat createSegmentationDisplay(cv::Mat & segments, int numOfSegments, cv::Mat & image);

public:
  djiScissor();
  ~djiScissor();

  int cut(cv::Mat& src_img, cv::Mat& segments);
};

#endif
