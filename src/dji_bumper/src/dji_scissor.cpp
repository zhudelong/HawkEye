#include "dji_bumper/dji_scissor.h"

using namespace cv;
using namespace std;

djiScissor::djiScissor()
{
}

djiScissor::~djiScissor()
{
}

void djiScissor::mergeSegments(Mat& image, Mat& segments, int& numOfSegments)
{
  //To collect pixels from each segment of the image
  vector<Mat> samples;
  //In case of multiple merging iterations, the numOfSegments should be updated
  int newNumOfSegments = numOfSegments;

  //Initialize the segment samples
  for (int i = 0; i <= numOfSegments; i++)
    {
      Mat sampleImage;
      samples.push_back(sampleImage);
    }

  //collect pixels from each segments
  for (int i = 0; i < segments.rows; i++ )
    {
      for (int j = 0; j < segments.cols; j++ )
        {
          //check what segment the image pixel belongs to
          int index = segments.at<int>(i, j);

          if (index >= 0 && index < numOfSegments)
            {
              samples[index].push_back(image(Rect(j, i, 1, 1)));
            }
        }
    }

  //create histograms
  vector<MatND> hist_bases;
  Mat hsv_base;
  /// Using 35 bins for hue component
  int h_bins = 35;
  /// Using 30 bins for saturation component
  int s_bins = 30;
  int histSize[] = { h_bins, s_bins };
  // hue varies from 0 to 256, saturation from 0 to 180
  float h_ranges[] = { 0, 256 };
  float s_ranges[] = { 0, 180 };
  const float* ranges[] = { h_ranges, s_ranges };
  // Use the 0-th and 1-st channels
  int channels[] = { 0, 1 };
  // To store the histograms
  MatND hist_base;

  for (int c = 1; c < numOfSegments; c++)
    {
      if (samples[c].dims > 0)
        {
          //convert the sample to HSV
          cvtColor( samples[c], hsv_base, CV_BGR2HSV );
          //calculate the histogram
          calcHist( &hsv_base, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false );
          //normalize the histogram
          normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );
          //append to the collection
          hist_bases.push_back(hist_base);
        }

      else
        {
          hist_bases.push_back(MatND());
        }

      hist_base.release();
    }

  //To store the similarity of histograms
  double similarity = 0;
  //to keep the track of already merged segments
  vector<bool> mearged;

  //initialize the merged segments tracker
  for (int k = 0; k < hist_bases.size(); k++)
    {
      mearged.push_back(false);
    }

  //calculate the similarity of the histograms of each pair of segments
  for (int c = 0; c < hist_bases.size(); c++)
    {
      for (int q = c + 1; q < hist_bases.size(); q++)
        {
          //if the segment is not merged alreay
          if (!mearged[q])
            {
              if (hist_bases[c].dims > 0 && hist_bases[q].dims > 0)
                {
                  //calculate the histogram similarity
                  similarity = compareHist(hist_bases[c], hist_bases[q], CV_COMP_BHATTACHARYYA);

                  //if similay
                  if (similarity > 0.8)
                    {
                      mearged[q] = true;

                      if (q != c)
                        {
                          //reduce number of segments
                          newNumOfSegments--;

                          for (int i = 0; i < segments.rows; i++ )
                            {
                              for (int j = 0; j < segments.cols; j++ )
                                {
                                  int index = segments.at<int>(i, j);

                                  //merge the segment q with c
                                  if (index == q)
                                    {
                                      segments.at<int>(i, j) = c;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

  numOfSegments = newNumOfSegments;
}

Mat djiScissor::watershedSegment(Mat& image, int& noOfSegments)
{
  //To store the gray version of the image
  Mat gray;
  //To store the thresholded image
  Mat ret;
  //convert the image to grayscale
  cvtColor(image, gray, CV_BGR2GRAY);
  imshow("Gray Image", gray);
  //threshold the image
  threshold(gray, ret, 0, 255, CV_THRESH_BINARY_INV + CV_THRESH_OTSU);
  imshow("Image after OTSU Thresholding", ret);
  //Execute morphological-open
  morphologyEx(ret, ret, MORPH_OPEN, Mat::ones(9, 9, CV_8SC1), Point(4, 4), 2);
  imshow("Thresholded Image after Morphological open", ret);
  //get the distance transformation
  Mat distTransformed(ret.rows, ret.cols, CV_32FC1);
  distanceTransform(ret, distTransformed, CV_DIST_L2, 3);
  //normalize the transformed image in order to display
  normalize(distTransformed, distTransformed, 0.0, 1, NORM_MINMAX);
  imshow("Distance Transformation", distTransformed);
  //threshold the transformed image to obtain markers for watershed
  threshold(distTransformed, distTransformed, 0.1, 1, CV_THRESH_BINARY);
  //Renormalize to 0-255 to further calculations
  normalize(distTransformed, distTransformed, 0.0, 255.0, NORM_MINMAX);
  distTransformed.convertTo(distTransformed, CV_8UC1);
  imshow("Thresholded Distance Transformation", distTransformed);
  //calculate the contours of markers
  int i, j, compCount = 0;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(distTransformed, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  if ( contours.empty() )
    return Mat();

  Mat markers(distTransformed.size(), CV_32S);
  markers = Scalar::all(0);
  int idx = 0;

  //draw contours
  for ( ; idx >= 0; idx = hierarchy[idx][0], compCount++ )
    drawContours(markers, contours, idx, Scalar::all(compCount + 1), -1, 8, hierarchy, INT_MAX);

  if ( compCount == 0 )
    return Mat();

  //calculate the time taken to the watershed algorithm
  double t = (double)getTickCount();
  //apply watershed with the markers as seeds
  watershed( image, markers );
  t = (double)getTickCount() - t;
  printf( "execution time = %gms\n", t * 1000. / getTickFrequency() );
  //create displayable image of segments
  Mat wshed = createSegmentationDisplay(markers, compCount, image);
  imshow( "watershed transform", wshed );
  noOfSegments = compCount;
  //returns the segments
  return markers;
}

Mat djiScissor::createSegmentationDisplay(Mat& segments, int numOfSegments, Mat& image)
{
  //create a new image
  Mat wshed(segments.size(), CV_8UC3);
  //Create color tab for coloring the segments
  vector<Vec3b> colorTab;

  for (int i = 0; i < numOfSegments; i++ )
    {
      int b = theRNG().uniform(0, 255);
      int g = theRNG().uniform(0, 255);
      int r = theRNG().uniform(0, 255);
      colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
    }

  //assign different color to different segments
  for (int i = 0; i < segments.rows; i++ )
    {
      for (int j = 0; j < segments.cols; j++ )
        {
          int index = segments.at<int>(i, j);

          if ( index == -1 )
            wshed.at<Vec3b>(i, j) = Vec3b(255, 255, 255);

          else if ( index <= 0 || index > numOfSegments )
            wshed.at<Vec3b>(i, j) = Vec3b(0, 0, 0);

          else
            wshed.at<Vec3b>(i, j) = colorTab[index - 1];
        }
    }

  //If the original image available then merge with the colors of segments
  if (image.dims > 0) wshed = wshed * 0.5 + image * 0.5;

  return wshed;
}

int djiScissor::cut(cv::Mat& src_img, cv::Mat& segments)
{
  std::cout << "djiScissor::cut() is called" << std::endl;
  //to store the number of segments
  int numOfSegments = 0;
  //Apply watershed
  segments = watershedSegment(src_img, numOfSegments);
  //Merge segments in order to reduce over segmentation
  mergeSegments(src_img, segments, numOfSegments);
  //To display the merged segments
  Mat wshed = createSegmentationDisplay(segments, numOfSegments, src_img);
  //To display the merged segments blended with the image
//  Mat wshedWithImage = createSegmentationDisplay(segments,numOfSegments,image);
  //Display the merged segments
  imshow("Merged segments", wshed);
  //Display the merged segments blended with the image
//  imshow("Merged segments with image",wshedWithImage);
  waitKey(1);
  return numOfSegments;
}
