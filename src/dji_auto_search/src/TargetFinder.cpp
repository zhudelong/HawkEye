#include "dji_auto_search/TargetFinder.h"
TargetFinder::TargetFinder(){
    bi_sig = 7;
    min_area = 600;
    sp = 19;
    sr = 40;
    sobel_ker_size = 5;
    median_ker_size = 5;
}


vector<RotatedRect> TargetFinder::get_selected_rects(vector<Mat>& edges){

    // sum up good quality channels
    Mat source = Mat::zeros(edges[0].size(),CV_8UC1);
    for(int i = 0; i < edges.size(); i++){
       cv::Mat element = getStructuringElement( MORPH_RECT,Size(3, 3));
       cv::dilate(edges[i],edges[i], element);
       cv::add(source, edges[i],source);
    }

    // find contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::findContours(source,contours,hierarchy,CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));

    // get polyDP
    vector<RotatedRect>  rects;
    for (size_t i = 0; i < contours.size(); i++){
        // if having father but son means this contour is
        if(hierarchy[i][2]!=-1 && hierarchy[i][3] == -1){
            continue;
        }

        // 1st cretia
        vector<Point> poly_temp;
        approxPolyDP(Mat(contours[i]), poly_temp, 3, true);
        if(poly_temp.size() < 3 || poly_temp.size() > 7){ // 3 7 PARA
            continue;
        }

        // 2nd cretia
        RotatedRect bound_rect_tmp = minAreaRect(Mat(poly_temp));
        float ratio = bound_rect_tmp.size.width / bound_rect_tmp.size.height;
        if(ratio > 1.7 || ratio < 0.4){
            continue;
        }

        // 3nd cretia
        float len = arcLength(Mat(contours[i]),true);
        float reg = contourArea(Mat(contours[i]));
        if(len < 100 || reg < 1000){
            continue;
        }

        // 4th cretia
        float cir = 2 * (bound_rect_tmp.size.width + bound_rect_tmp.size.height);
        float cir_rio = std::fabs(cir - len)/cir;
        float area_rio = std::fabs(bound_rect_tmp.size.area() - reg)/reg;
        if(cir_rio > 0.5 || area_rio > 0.5 ){
            continue;
        }
#ifdef SHOW_RESULT
        std::cout<<"rect area:" <<bound_rect_tmp.size.area();
        std::cout << "minArea:"<<contourArea(Mat(contours[i]))<<std::endl;
        std::cout << "ratio:"<<area_rio<<std::endl;
        std::cout << "rectLen:"<<cir;
        std::cout << "arcLeng:"<<arcLength(Mat(contours[i]),true)<<std::endl;
        std::cout << "ratio:"<<cir_rio<<std::endl;
#endif
        rects.push_back(bound_rect_tmp);
    }

//#ifdef SHOW_RESULT


    Mat drawing = Mat::zeros(source.size(), CV_8UC3);
    RNG rng(12345);
    for (size_t i = 0; i < rects.size(); i++)
    {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        //drawContours(drawing, contours_poly, (int) i, color, 1, 8, vector<Vec4i>(), 0, Point());
        Point2f rect_points[4];
        rects[i].points(rect_points);
        for( int j = 0; j < 4; j++ )
        {
            line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        }
    }
    imshow("ret",drawing);
//#endif
    return rects;
}

void TargetFinder::get_canny_channels(Mat& ori,vector<Mat>&edges ){
    gpu::GpuMat src_bgr(ori);

    vector<gpu::GpuMat> bgr;
    gpu::split(src_bgr,bgr);
    //    for(int i = 0; i < int(bgr.size()+1); i++){

    for(int i = 1; i < int(bgr.size()); i++){
        // segment 4 channels
        gpu::GpuMat seg_temp;
        Mat seg_ret; // the result of segmentation
        if(i == int(bgr.size())){// process original image
            gpu::cvtColor(src_bgr,seg_temp,CV_BGR2BGRA);
            gpu::meanShiftSegmentation(seg_temp,seg_ret,sp,sr,min_area);
            src_bgr.release();
        }else{// process the thre channels separately
            gpu::cvtColor(bgr[i],seg_temp,CV_GRAY2BGRA);
            gpu::meanShiftSegmentation(seg_temp,seg_ret,sp,sr,min_area);
        }

        // convert to gray and filtering
        Mat seg_gray;
        cv::cvtColor(seg_ret,seg_gray,CV_BGRA2GRAY);
        cv::medianBlur(seg_gray,seg_gray,median_ker_size);
        seg_ret.release();

        double value = this->get_median(seg_gray);
        int low = int(std::max(0.0,  (1.0 - 0.33) * value));
        int up = int(std::min(255.0, (1.0 + 0.33) * value));

        // get sobel edges
        gpu::GpuMat canny_edge;
        gpu::GpuMat seg_gray_gpu(seg_gray);
        gpu::Canny(seg_gray_gpu, canny_edge, low, up);
        seg_gray_gpu.release();
        edges.push_back(Mat(canny_edge));

#ifdef SHOW_RESULT
        char name[20];
        sprintf(name, "canny%d",i);
        imshow(name,Mat(seg_temp));
        waitKey(1);
#endif
    }
}

vector<Mat> TargetFinder::get_sobel_channels(Mat& ori){
    gpu::GpuMat src_bgr(ori);
    vector<gpu::GpuMat> bgr;
    gpu::split(src_bgr,bgr);
    vector<Mat> edges;
    for(int i = 0; i < int(bgr.size()+1); i++){
        // segment 4 channels
        gpu::GpuMat seg_temp;
        Mat seg_ret; // the result of segmentation
        if(i == bgr.size()){// process original image
            gpu::cvtColor(src_bgr,seg_temp,CV_BGR2BGRA);
            gpu::meanShiftSegmentation(seg_temp,seg_ret,sp,sr,min_area);
            src_bgr.release();
        }else{// process the thre channels separately
            gpu::cvtColor(bgr[i],seg_temp,CV_GRAY2BGRA);
            gpu::meanShiftSegmentation(seg_temp,seg_ret,sp,sr,min_area);
        }

        // convert to gray and filtering
        Mat seg_gray;
        cv::cvtColor(seg_ret,seg_gray,CV_BGRA2GRAY);
        cv::medianBlur(seg_gray,seg_gray,median_ker_size);
        seg_ret.release();

        // get sobel edges
        gpu::GpuMat seg_gray_gpu(seg_gray);
        gpu::GpuMat grad_x, grad_y, grad_c;
        gpu::Sobel(seg_gray_gpu,grad_x,CV_16S,1,0,sobel_ker_size,1);
        gpu::Sobel(seg_gray_gpu,grad_y,CV_16S,0,1,sobel_ker_size,1);
        gpu::add(grad_x,grad_y,grad_c);
        seg_gray_gpu.release();
        grad_x.release();
        grad_y.release();
        Mat sobel_edge;
        cv::convertScaleAbs(Mat(grad_c),sobel_edge);
        grad_c.release();
        edges.push_back(sobel_edge);
#ifdef SHOW_RESULT

        char name[20];
        sprintf(name, "sobel%d",i);
        imshow(name,sobel_edge);
        waitKey(1);
#endif
    }
    return edges;
}


bool TargetFinder::set_gpu_device(int dev){
    int num_dev = gpu::getCudaEnabledDeviceCount();
    if(num_dev < 1){
        ROS_ERROR("No cuda deveice!");
        return false;
    }
    gpu::DeviceInfo dev_info(dev);
    if(!dev_info.isCompatible()){
        ROS_ERROR("GPU module isn't built for this GPU!");
        return false;
    }
    gpu::setDevice(dev);
    return true;
}

float TargetFinder::get_median(Mat& channel) const{
    double m = (channel.rows*channel.cols) / 2;
    int bin = 0;
    double med = -1.0;

    int histSize = 256;
    float range[] = { 0, 256 };
    const float* histRange = { range };
    bool uniform = true;
    bool accumulate = false;
    cv::Mat hist;
    cv::calcHist( &channel, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );

    for ( int i = 0; i < histSize && med < 0.0; ++i )
    {
        bin += cvRound( hist.at< float >( i ) );
        if ( bin > m && med < 0.0 )
            med = i;
    }
    return med;
}

vector<RotatedRect> TargetFinder::get_candidate_areas(cv::Mat& ori_gray)
{
    vector<RotatedRect> rects;
    if(ori_gray.empty() == true)
    {
        ROS_ERROR("get_candidate_areas:image is empty!");
        return rects;
    }
    Mat src;
    if(ori_gray.channels()!= 1){
        ROS_WARN("get_candidate_areas:: convert color to gray!");
        cv::cvtColor(ori_gray,src,CV_BGR2GRAY);
    }

    // find potential contours
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    cv::blur(src, src,cv::Size(3,3));
    double value = this->get_median(src);
    int low = int(std::max(0.0,  (1.0 - 0.33) * value));
    int up = int(std::min(255.0, (1.0 + 0.33) * value));
    cv::Canny(src, canny_output, low, up * 2, 3);

    // dilate the edge to find more candidates
    cv::Mat element = getStructuringElement( MORPH_RECT,Size(3, 3), Point(1, 1));
    cv::dilate(canny_output,canny_output, element);

    // find contours
    cv::findContours(canny_output, contours, hierarchy, CV_RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));
    for (size_t i = 0; i < int (contours.size()); i++)
    {
        // if having father but son means this contour is
        if(hierarchy[i][2]!=-1 && hierarchy[i][3] == -1){
            continue;
        }

        vector<Point> poly_temp;
        approxPolyDP(Mat(contours[i]), poly_temp, 3, true);
        if(poly_temp.size() < 4 || poly_temp.size() > 7 ){
            continue;
        }

        // 2nd cretia
        RotatedRect bound_rect_tmp = minAreaRect(Mat(poly_temp));
        float ratio = bound_rect_tmp.size.width / bound_rect_tmp.size.height;
        if(ratio > 1.7 || ratio < 0.4){
            continue;
        }

        // 3nd cretia
        float len = arcLength(Mat(contours[i]),true);
        float reg = contourArea(Mat(contours[i]));
        if(len < 100 || reg < 1000){
            continue;
        }

        // 4th cretia
        float cir = 2 * (bound_rect_tmp.size.width + bound_rect_tmp.size.height);
        float cir_rio = std::fabs(cir - len)/cir;
        float area_rio = std::fabs(bound_rect_tmp.size.area() - reg)/reg;
        if(cir_rio > 0.5 || area_rio > 0.5 ){
            continue;
        }
#ifdef SHOW_RESULT

        std::cout<<"rect area:" <<bound_rect_tmp.size.area();
        std::cout << "minArea:"<<contourArea(Mat(contours[i]))<<std::endl;
        std::cout << "ratio:"<<area_rio<<std::endl;
        std::cout << "rectLen:"<<cir;
        std::cout << "arcLeng:"<<arcLength(Mat(contours[i]),true)<<std::endl;
        std::cout << "ratio:"<<cir_rio<<std::endl;
#endif
        rects.push_back(bound_rect_tmp);
    }

#ifdef SHOW_RESULT

    // show result---find contour will change the canny image;
    RNG rng1(12345);
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    for (size_t i = 0; i < int(rects.size()); i++)
    {
        Scalar color = Scalar(rng1.uniform(0, 255), rng1.uniform(0, 255), rng1.uniform(0, 255));
        drawContours(drawing, contours, (int) i, color, 1, 8, vector<Vec4i>(), 0, Point());
        Point2f rect_points[4];
        rects[i].points(rect_points);
        for( int j = 0; j < 4; j++ )
        {
            line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        }
    }
    imshow("get_candidate_areas:canny",canny_output);
    imshow("get_candidate_areas:ret",drawing);
#endif
    return rects;
}





