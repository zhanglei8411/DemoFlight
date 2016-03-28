#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/gpu/gpumat.hpp"
#include "opencv2/video/tracking.hpp"
#include "fisheye.hpp"
#include <string>
#include <iostream>
#include "getTargetLoc.hpp"
#include "inifile.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <fstream>
using namespace cv;
using namespace std;

#define  BUF_SIZE 256

/** function pro = kx-y+j, take two points a and b,
*** compute the line argument k and j, then return the pro value
*** so that can be used to determine whether the point p is on the left or right
*** of the line ab
**/
static double computeProduct(Point2f p, Point2f a, Point2f b)
{
    double k, j;
    if (a.x == b.x)
    {
        j = -a.x;
        return p.x + j;
    }
    else
    {
        k = (a.y-b.y) / (a.x-b.x);
        j = a.y - k*a.x;
        return k*p.x - p.y + j;
    }
}

/** decide whether point p is in the ROI.
*** The ROI is a rotated rectange whose 4 corners are stored in roi[]
**/
static bool isInROI(cv::Point2f p, cv::Point2f roi[])
{
    double pro[4];
    for(int i=0; i<4; ++i)
    {
        pro[i] = computeProduct(p, roi[i], roi[(i+1)%4]);
    }
    if(pro[0]*pro[2]<0 && pro[1]*pro[3]<0)
    {
        return true;
    }
    return false;
}

static bool isBoxOnBoundary(cv::RotatedRect box, cv::Size imgSize, int thre)
{
    if (box.center.x - box.size.width< thre || box.center.x + box.size.width > imgSize.width - thre ||
        box.center.y - box.size.height < thre || box.center.y + box.size.height > imgSize.height - thre)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static double max(double a, double b)
{
    if (a > b)
        return a;
    else
        return b;
}
static double min(double a, double b)
{
    if(a < b)
        return a;
    else
        return b;
}

 cv::Mat xyVision::GetTarget::equalizeIntensity(const Mat& inputImage)
{
    if(inputImage.channels() >= 3)
    {
        Mat ycrcb;

        cvtColor(inputImage,ycrcb,CV_BGR2Lab);

        vector<Mat> channels;
        split(ycrcb,channels);

        equalizeHist(channels[0], channels[0]);
        //Mat1b out;
        //imadjust(channels[0], out);
        //channels[0] = out.clone();

        Mat result;
        merge(channels,ycrcb);

        cvtColor(ycrcb,result,CV_Lab2BGR);

        return result;
    }
    return Mat();
}


xyVision::GetTarget::GetTarget(string configName)
{
    this->setCameraParams(configName);
    frameCounter = 0;
    frameCounterTarget = 0;
    isDetected = true;
    this->useGPU = false;
    this->timeModel = DAYTIME;
    time1 = 0;
    time2 = 0;
    time3 = 0;


    if (this->cameraInfo.cameraType == 1)
    {
        cameraInfo.newCameraMatrix = cameraInfo.cameraMatrix * proInfo.scale;
        cameraInfo.newCameraMatrix(2, 2) = 1.0f;
    }
        // compute rectify mappings
    else if (this->cameraInfo.cameraType == 2)
    {
        xyVision::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraInfo.cameraMatrix, cameraInfo.distortCoeff,
            cameraInfo.imageSize, cv::Matx33d::eye(), cameraInfo.newCameraMatrix, 0.8, cameraInfo.newSize, 1);
        xyVision::fisheye::initUndistortRectifyMap(cameraInfo.cameraMatrix, cameraInfo.distortCoeff,
            cv::Matx33d::eye(), cameraInfo.newCameraMatrix, cameraInfo.newSize, CV_16SC2, cameraInfo.map1, cameraInfo.map2);
    }

    // initialize Kalman fileter
    KF = KalmanFilter(6, 3, 0);
    KF.transitionMatrix = *(Mat_<float>(6,6) << 1,0,0,1,0,0,   0,1,0,0,1,0,  0,0,1,0,0,1,  0,0,0,1,0,0,  0,0,0,0,1,0,  0,0,0,0,0,1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(0.5));
    setIdentity(KF.errorCovPost, Scalar::all(1e-2));

    // compute mapping
    this->mapping.resize(256);
    double p[5] = {-3.283780259159030e-07, 1.182791454914262e-04, -0.006030084747775, 0.254476804888576, 2.376613986565857};
    for (int i = 0; i < 256; ++i)
    {
        double r = 0;
        for (int o = 4; o >=0; o--)
        {
            r = r + p[4-o]*pow((double)i, (double)o);
        }
        r = max(r, 0);
        r = min(255, r);
        r = int(floor(r));
        mapping[i] = (int)r;
    }
}

void xyVision::GetTarget::setCameraParams(string configFileName)
{
    // read camera parameters
    this->configFileName = configFileName;
    const char * section = "cameraConfig";
    const char * key = "cameraType";
    this->cameraInfo.cameraType = read_profile_int(section, key, 0, configFileName.c_str());
    key = "fx";
    float fx = read_profile_float(section, key, 0, configFileName.c_str());
    key = "fy";
    float fy = read_profile_float(section, key, 0, configFileName.c_str());
    key = "u0";
    float u0 = read_profile_float(section, key, 0, configFileName.c_str());
    key = "v0";
    float v0 = read_profile_float(section, key, 0, configFileName.c_str());
    key = "k1";
    float k1 = read_profile_float(section, key, 0, configFileName.c_str());
    key = "k2";
    float k2 = read_profile_float(section, key, 0, configFileName.c_str());
    key = "k3";
    float k3 = read_profile_float(section, key, 0, configFileName.c_str());
    key = "k4";
    float k4 = read_profile_float(section, key, 0, configFileName.c_str());
    key = "imageWidth";
    int imageWidth = read_profile_int(section, key, 0, configFileName.c_str());
    key = "imageHeight";
    int imageHeight = read_profile_int(section, key, 0, configFileName.c_str());

    Matx33f _cameraMat(fx, 0, u0, 0, fy, v0, 0, 0, 1);
    Matx14f _dis(k1, k2, k3, k4);
    this->cameraInfo.cameraMatrix = _cameraMat;
    this->cameraInfo.distortCoeff = _dis;
    this->cameraInfo.imageSize = Size(imageWidth, imageHeight);

    // read target information
    section = "targetConfig";
    key = "width";
    float width = read_profile_float(section, key, 0, configFileName.c_str());
    key = "version";
    int version = read_profile_int(section, key, 0, configFileName.c_str());
    key = "targetVersionFile";
    char versionFile[100];
    read_profile_string(section, key, versionFile, 100, "-1", configFileName.c_str());
    float widthInner;
    if (version == 2)
    {
        key = "innerWidth";
        widthInner = read_profile_float(section, key, 0, configFileName.c_str());
    }

    this->targetInfo.targetWidth = width;
    this->targetInfo.targetVersion = version;
    this->oriTargetVersion = version;
    this->versionFileName = versionFile;
    if (version == 2)
    {
        this->targetInfo.targetWidthInner = widthInner;
    }

    //if (version == 3)
    //{
        key = "n_aruco";
        int n_aruco = read_profile_int(section, key, 0, configFileName.c_str());
        key = "dic_id";
        int dic_id = read_profile_int(section, key, 0, configFileName.c_str());
        if (dic_id == 0)
        {
            this->targetInfo.board.dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
        }
        this->targetInfo.board.objPoints.resize(n_aruco);
        for (int i = 0; i < n_aruco;  ++i)
        {
            this->targetInfo.board.ids.push_back(i);
            for (int p = 0; p < 4; p++)
            {
                char key2[20];
                sprintf(key2, "id_%d_p_%d_x", i, p);
                float x = read_profile_float(section, key2, 0, configFileName.c_str());
                sprintf(key2, "id_%d_p_%d_y", i, p);
                float y = read_profile_float(section, key2, 0, configFileName.c_str());
                this->targetInfo.board.objPoints[i].push_back(Point3f(x, y, 0.0f));
            }
        }
//    }

    // read processing information
    section = "processingConfig";
    key = "scale";
    float scale = read_profile_float(section, key, 0, configFileName.c_str());
    this->proInfo.scale = scale;

    this->cameraInfo.newSize = Size(int(imageWidth*scale), int(imageHeight*scale));

    cout << "camera matrix " << endl;
    cout << this->cameraInfo.cameraMatrix << endl;
    cout << "image size" << endl;
    cout << this->cameraInfo.imageSize.width << " " << this->cameraInfo.imageSize.height << endl;
    cout << "new image size" << endl;
    cout << this->cameraInfo.newSize.width << " " << this->cameraInfo.newSize.height << endl;

}

xyVision::GetTarget& xyVision::GetTarget::operator<<(const Mat& image)
{
    if (this->frameCounter > 0)
    {
        this->previousImg = this->currentImg.clone();
    }
    this->currentImg = image.clone();

    if (this->useGPU)
    {
        this->runOneFrame_gpu();
    }
    else
    {
        this->runOneFrame();
    }

    if (isDetected)
    {
        // run kalman filter here
        // first frame, initialize kalman filter
        Point3f _point = this->getCurrentLoc();
        if (this->frameCounterTarget == 0)
        {

            KF.statePre = Mat(Matx16f(_point.x, _point.y, _point.z, 0.f, 0.f, 0.f));
        }
        else if (this->frameCounterTarget > 0)
        {
            KF.predict();
            Mat estimate = KF.correct(Mat(_point));
            //this->currentLoc = Point3f(estimate.at<float>(0), estimate.at<float>(1), estimate.at<float>(2));
        }
        this->frameCounterTarget ++;
    }
    this->frameCounter ++;
    return *this;
}

Point3f xyVision::GetTarget::getCurrentLoc()
{
    return this->currentLoc;
}

vector<Point3f> xyVision::GetTarget::getAllLoc()
{
    return this->allLoc;
}

void xyVision::GetTarget::imadjust(const Mat1b& src, Mat1b& dst, int tol, Vec2i in, Vec2i out)
{
    dst = src.clone();
    tol = max(0, min(100, tol));
    if (tol > 0)
    {
        // Compute in and out limits
        // Histogram
        vector<int> hist(256, 0);
        for (int r = 0; r < src.rows; ++r) {
            for (int c = 0; c < src.cols; ++c) {
                hist[src(r,c)]++;
            }
        }
        // Cumulative histogram
        vector<int> cum = hist;
        for (int i = 1; i < (int)hist.size(); ++i) {
            cum[i] = cum[i - 1] + hist[i];
        }
        // Compute bounds
        int total = src.rows * src.cols;
        int low_bound = total * tol / 100;
        int upp_bound = total * (100-tol) / 100;
        in[0] = (int)distance(cum.begin(), lower_bound(cum.begin(), cum.end(), low_bound));
        in[1] = (int)distance(cum.begin(), lower_bound(cum.begin(), cum.end(), upp_bound));
    }
    // Stretching
    float scale = float(out[1] - out[0]) / float(in[1] - in[0]);
    for (int r = 0; r < dst.rows; ++r)
    {
        for (int c = 0; c < dst.cols; ++c)
        {
            int vs = max(src(r, c) - in[0], 0);
            int vd = min(int(vs * scale + 0.5f) + out[0], out[1]);
            dst(r, c) = saturate_cast<uchar>(vd);
        }
    }
}

void xyVision::GetTarget::adjustImg(Mat & img)
{
    CV_Assert(img.channels() == 3);
    // if (img.channels() != 3)
    // {
    // 	printf("img channel is not 3 \n");
    // 	return ;
    // }
    //img = equalizeIntensity(img);
    //imwrite("img.jpg", img);
    Size sz = img.size();
    float scaleFactor = this->proInfo.scale;
    if (cameraInfo.cameraType == 1)
    {
        cv::resize(img, img, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
    }
    else
    {
        cv::remap(img, img, cameraInfo.map1, cameraInfo.map2, INTER_LINEAR, BORDER_CONSTANT);
        //img = equalizeIntensity(img);

        this->rectified = img.clone();
    }
    //vector<Mat> chs;
    //split(img, chs);
    //Mat1b out;
    // b g r
    //imadjust(chs[2], out);
    //imadjust_mapping(chs[2], out, this->mapping);
    //cv::equalizeHist(chs[2], out);
    //chs[2] = out.clone();

    //imadjust(chs[1], out);
    //imadjust_mapping(chs[1], out, this->mapping);
    //chs[1] = out.clone();
    //imadjust(chs[0], out);
    //imadjust_mapping(chs[0], out, this->mapping);
    //chs[0] = out.clone();

    //merge(chs, img);
    //cv::imwrite("adj.bmp", img);
}

void xyVision::GetTarget::adjustImg_gpu(gpu::GpuMat& img)
{
    CV_Assert(img.channels() == 3);

    Size sz = img.size();
    float scaleFactor = this->proInfo.scale;
    gpu::GpuMat img2;
    gpu::resize(img, img2, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
    img = img2.clone();

	gpu::getCudaEnabledDeviceCount();
	gpu::setDevice(0);
	vector<gpu::GpuMat> chs;
	gpu::split(img, chs);
	gpu::GpuMat out;
	// b g r
	//imadjust(chs[2], out);
	gpu::equalizeHist(chs[2], out);
	chs[2] = out;
	gpu::merge(chs, img);
}

void xyVision::GetTarget::imadjust_mapping(const Mat& src, Mat& dst, vector<int> mapping)
{
    CV_Assert(src.type() == CV_8UC1);
    dst = src.clone();
    const unsigned char* pt_src = src.ptr<unsigned char>(0);
    unsigned char* pt_dst = dst.ptr<unsigned char>(0);
    int n = (int)src.total();
    for (int i = 0; i < n; ++i)
    {
        pt_dst[i] = this->mapping[(int)pt_src[i]];
    }
}

void xyVision::GetTarget::binarizeTarget(const Mat & img, Mat & bi)
{
    CV_Assert(img.channels() == 3);

    std::vector<Mat> chs;

    Mat img2 = img.clone();
    //img2.convertTo(img2, CV_32FC3);
    split(img2, chs);
    Mat img_t = chs[2] - chs[0] - chs[1];
    cv::threshold(img_t, bi, 30, 255, THRESH_BINARY);
    bi.convertTo(bi, CV_8UC1);
    cv::imwrite("bi.bmp", bi);
    // resize binary image
    //Size sz = img2.size();
    //float scaleFactor = this->proInfo.scale;
    //resize(bi, bi, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));

    // improve binary image
    //Mat kernel = cv::Mat::ones(3, 3, CV_8UC1);
    //dilate(bi, bi, kernel);
    //erode(bi, bi, kernel);

}

void xyVision::GetTarget::binarizeTarget_gpu(const gpu::GpuMat img, gpu::GpuMat & bi)
{
    CV_Assert(img.channels() == 3);
    gpu::GpuMat img2 = img.clone();
    std::vector<gpu::GpuMat> chs;
    gpu::split(img2, chs);

    Mat tmp;

    //gpu::GpuMat img_t = chs[2] - chs[0] - chs[1];
    gpu::GpuMat img_t, img_tmp;
    gpu::subtract(chs[2], chs[0], img_tmp);
    gpu::subtract(img_tmp, chs[1], img_t);

    gpu::threshold(img_t, bi, 40, 255, THRESH_BINARY);

    bi.convertTo(bi, CV_8UC1);
}
void xyVision::GetTarget::binarizeTarget_HSV(const Mat& img, Mat & bi)
{
    Mat img_hsv;
    cvtColor(img, img_hsv, CV_BGR2HSV);
    std::vector<Mat> chs;
    Mat img2 = img.clone();
    img2.convertTo(img2, CV_32F);
    split(img2, chs);

    Mat img_t = chs[2] - chs[0] - chs[1];
    split(img2, chs);

    // H is 0-180, 221-255 is red
    Mat bi_hsv1, bi_hsv2,bi_rgb;

    inRange(img_hsv, Scalar(160, 40, 40), Scalar(180, 255, 255), bi_hsv1);
    //inRange(img_hsv, Scalar(0, 40, 40), Scalar(20, 255, 255), bi_hsv2);
    cv::threshold(chs[2], bi_rgb, 100, 255, THRESH_BINARY);
    //inRange(img, Scalar(0, 0, 100), Scalar(200, 200, 255), bi_rgb);
    //inRange(img, Scalar(0, 0, 0), Scalar(255, 255, 255), bi_rgb);
    //cv::imwrite("bi_hsv1.bmp", bi_hsv1);
    //cv::imwrite("bi_hsv2.bmp", bi_hsv2);
    //cv::imwrite("bi_rgb.bmp", bi_rgb);
    bi_hsv1.convertTo(bi_hsv1, CV_32F);
    //bi_hsv2.convertTo(bi_hsv2, CV_32F);
    bi_rgb.convertTo(bi_rgb, CV_32F);
    bi_hsv1 = bi_hsv1 / 255;
    //bi_hsv2 = bi_hsv2 / 255;
    bi_rgb = bi_rgb / 255;

    //bi = bi_hsv1.mul(bi_rgb).mul(bi_hsv2) * 255;
    cv::threshold((bi_hsv1).mul(bi_rgb), bi, 0, 255, THRESH_BINARY);
    bi.convertTo(bi, CV_8U);


    Mat kernel = cv::Mat::ones(5, 5, CV_8UC1);
    dilate(bi, bi, kernel);
    erode(bi, bi, kernel);
    //cv::imwrite("bi.bmp", bi);
}

void xyVision::GetTarget::binarizeTarget_LAB(const Mat& img, Mat & bi)
{

    Mat img_lab;
    cvtColor(img, img_lab, CV_BGR2Lab);

    Mat img_hsv;
    cvtColor(img, img_hsv, CV_BGR2HSV);

    Mat img_luv;
    cvtColor(img, img_luv, CV_BGR2Luv);

    std::vector<Mat> chs;
    Mat img2 = img.clone();
    img2.convertTo(img2, CV_32F);
    split(img2, chs);

    Mat bi_lab, bi_rgb, bi_hsv1, bi_hsv2, bi_hsv, bi_luv;
    inRange(img_lab, Scalar(100, 140, 0), Scalar(255, 255, 255), bi_lab);
    inRange(img_hsv, Scalar(160, 40, 40), Scalar(180, 255, 255), bi_hsv1);
    inRange(img_hsv, Scalar(0, 40, 40), Scalar(20, 255, 255), bi_hsv2);
    inRange(img_luv, Scalar(100, 110, 110), Scalar(255, 255, 255), bi_luv);
    bi_hsv = bi_hsv1 + bi_hsv2;
    //inRange(img_lab, Scalar(0, 130, 0), Scalar(255, 255, 255), bi_lab);
    //cv::threshold(chs[2], bi_rgb, 100, 255, THRESH_BINARY);
    inRange(img, Scalar(0,0,150), Scalar(200, 200, 255), bi_rgb);

    //cv::imwrite("bi_lab.jpg", bi_lab);
    //cv::imwrite("bi_rgb.jpg", bi_rgb);
    //cv::imwrite("bi_hsv.jpg", bi_hsv);
    //cv::imwrite("bi_luv.jpg", bi_luv);

    bi_lab.convertTo(bi_lab, CV_32F);
    bi_rgb.convertTo(bi_rgb, CV_32F);
    bi_hsv.convertTo(bi_hsv, CV_32F);
    bi_luv.convertTo(bi_luv, CV_32F);


    cv::threshold(bi_lab.mul(bi_rgb).mul(bi_hsv).mul(bi_luv), bi, 0, 255, THRESH_BINARY);
    //cv::threshold(bi_luv, bi, 0, 255, THRESH_BINARY);
    bi.convertTo(bi, CV_8U);

    int size_ker = 2;
    if (this->targetInfo.targetVersion == 1)
    {
        size_ker = 4;
        if (frameCounterTarget > 1 && lastLoc.z < 1000)
            size_ker = 10;
    }
    //int size_ker = 5;
    //if (frameCounterTarget > 1 && lastLoc.z < 1000)
    //    size_ker = 10;

    Mat kernel = cv::Mat::ones(size_ker, size_ker, CV_8UC1);
    dilate(bi, bi, kernel);
    erode(bi, bi, kernel);
    //cv::imwrite("bi.jpg", bi);
}

void xyVision::GetTarget::binarizeTarget_night(const Mat& img, Mat & bi)
{
    Mat img_gray;
    cvtColor(img, img_gray, CV_BGR2GRAY);
    //Mat bi_gray;
    inRange(img_gray, Scalar(220), Scalar(255), bi);
    bi.convertTo(bi, CV_8UC1);
    //cv::imwrite("bi.bmp", bi);
}

bool xyVision::GetTarget::contourDetect(Mat& bi, vector<Point> & tarContour, vector<vector<Point> > & tarContours)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Size sz = bi.size();

    //namedWindow("bi");
    //imshow("bi", bi);
    //waitKey(0);

    findContours(bi.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    //cout << "time for finding Contours " << totaltime << endl;
    // no contours
    if ((int)contours.size() == 0)
        return false;

    vector<vector<Point> > contours_filter;
    std::vector<double> contours_areas_filter;
    std::vector<double> center_dis_filter;
    std::vector<double> ratio_filter;
    std::vector<RotatedRect> boxs_filter;

    // filter by areas
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        double area = contourArea(contours[i]);
        if (area > (sz.width/80.0f*sz.height/80.0f))
        {
            contours_filter.push_back(contours[i]);
        }
    }
    contours = contours_filter;
    contours_filter.resize(0);

    // filter by square
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        RotatedRect box = minAreaRect(Mat(contours[i]));
        float height = float(box.size.height);
        float width = float(box.size.width);
        float thre = 1.5f;
        // width/hight ratio is not constrained when target is near
        if (frameCounterTarget > 0 && lastLoc.z < 300)
        {
            thre = 10;
        }
        if (height / width < thre && width / height < thre)
        {
            contours_filter.push_back(contours[i]);
        }
    }
    contours = contours_filter;
    contours_filter.resize(0);

    // filter by fill in ratio
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        RotatedRect box = minAreaRect(Mat(contours[i]));
        double aera = contourArea(contours[i]);
        double ratio = computeRotatedRect(contours[i], sz, box, bi);
        if ((ratio > 0.7 && timeModel == DAYTIME) || (ratio < 0.2 && timeModel == NIGHT))
        {
            contours_filter.push_back(contours[i]);
            boxs_filter.push_back(box);
            contours_areas_filter.push_back(aera);
            ratio_filter.push_back(ratio);
        }
    }

    contours = contours_filter;

    if ((int)contours.size() == 0)
    {
        return false;
    }

    // choose the max one
    int choosen_idx = 0;
    int maxIdx = 0;
    double maxAera = contours_areas_filter[0];
    for (int i = 1; i < (int)contours.size(); ++i)
    {
        double conAera_i = contours_areas_filter[i];
        if (conAera_i > maxAera)
        {
            maxAera = conAera_i;
            maxIdx = i;
        }
    }
    choosen_idx = maxIdx;

    double choosen_aera = contours_areas_filter[choosen_idx];

    //if (choosen_aera < (sz.width/80.0f*sz.height/80.0f))
    //	return false;
    if (this->frameCounterTarget > 0 &&
        ((double)choosen_aera / (double)this->lastArea < 3.0/10 ))
    {
        return false;
    }
    // set zeros for other contours
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        if (i != choosen_idx)
        {
            drawContours(bi, contours, i, cv::Scalar(0), CV_FILLED, 8);
            //drawContours(img_t, contours, i, cv::Scalar(0), CV_FILLED, 8);
        }
    }
    tarContour = contours[choosen_idx];
    this->lastArea = choosen_aera;
    tarContours = contours;
    return true;
}

bool xyVision::GetTarget::contourDetect_day_tar_ver2(const Mat& img,
    vector<Point>& tarContour_outter, vector<Point>& tarContour_inner)
{
    // Step 1: get outter rect
    Mat bi1;
    binarizeTarget_LAB(img, bi1);
    //namedWindow("bi");
    //imshow("bi", bi1);
    //waitKey(0);
    // Step 2: get inner rect based on outer rect
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Size sz = bi1.size();
    findContours(bi1.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if ((int)contours.size() == 0)
        return false;

        //filter
    vector<vector<Point> > contours_filter1;
    std::vector<RotatedRect> boxs_filter1;

        // filter by areas
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        double area = contourArea(contours[i]);
        if (area > (sz.width/80.0f*sz.height/80.0f))
        {
            contours_filter1.push_back(contours[i]);
        }
    }
    contours = contours_filter1;
    contours_filter1.resize(0);

        // filter by square
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        RotatedRect box = minAreaRect(Mat(contours[i]));
        float height = float(box.size.height);
        float width = float(box.size.width);
        float thre = 1.3f;
        // width/hight ratio is not constrained when target is near
        if (frameCounterTarget > 0 && lastLoc.z < 300)
        {
            thre = 10;
        }
        if (height / width < thre && width / height < thre)
        {
            contours_filter1.push_back(contours[i]);
        }
    }
    contours = contours_filter1;
    contours_filter1.resize(0);
    vector<RotatedRect> boxs_filter1_tmp;
        // filter by fill in ratio
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        RotatedRect box = minAreaRect(Mat(contours[i]));
        contourArea(contours[i]);
        double ratio = computeRotatedRect(contours[i], sz, box, bi1);
        if (ratio < 0.9 && ratio > 0.3)
        {
            contours_filter1.push_back(contours[i]);
            boxs_filter1_tmp.push_back(box);
        }
    }
    contours = contours_filter1;
    contours_filter1.resize(0);

    vector<vector<Point> > contours_filter2;
    vector<RotatedRect> boxs_filter2;

    for (int i = 0; i < (int)contours.size(); ++i)
    {
        Rect rect = boundingRect(contours[i]);
        Mat roi = Mat(rect.height, rect.width, CV_8UC3);
        img(rect).copyTo(roi);

        Mat roi_lab;
        cvtColor(roi, roi_lab, CV_BGR2Lab);
        Mat roi_bi;
        inRange(roi_lab, Scalar(200, 0, 0), Scalar(255, 255, 255), roi_bi);

        //namedWindow("roi_bi");
        //imshow("roi_bi", roi);
        //waitKey(0);
        vector<vector<Point> > roi_contours;
        vector<Vec4i> roi_hierarchy;
        //Size roi_sz = roi_bi.size();
        Point offset((int)rect.x, (int)rect.y);
        findContours(roi_bi.clone(), roi_contours, roi_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


        if ((int)roi_contours.size()==0)
        {
            continue;
        }

        vector<vector<Point> > roi_contours_filter;
        // filter by fillin ratio
        for (int xx = 0; xx < (int)roi_contours.size(); ++xx)
        {
            RotatedRect box_roi = minAreaRect(Mat(roi_contours[xx]));
            contourArea(roi_contours[xx]);
            double ratio = computeRotatedRect(roi_contours[xx], roi_bi.size(), box_roi, roi_bi);
            if (ratio > 0.7)
            {
                roi_contours_filter.push_back(roi_contours[xx]);
            }
        }
        roi_contours = roi_contours_filter;
        if ((int)roi_contours_filter.size() == 0)
        {
            continue;
        }

        // find the largest one
        int roi_max_idx = 0;
        double roi_max_area = contourArea(roi_contours[0]);
        for (int j = 1; j < (int)roi_contours.size(); ++j)
        {
            double area = contourArea(roi_contours[j]);
            if (area > roi_max_area)
            {
                roi_max_area = area;
                roi_max_idx = j;
            }
        }
        RotatedRect box_inner = minAreaRect(Mat(roi_contours[roi_max_idx]));
        box_inner.center = box_inner.center + Point2f((float)rect.x,(float)rect.y);
        Point2f outer_extream_points[4];
        boxs_filter1_tmp[i].points(outer_extream_points);
        float max_len_inner = max(box_inner.size.width, box_inner.size.height);
        float max_len_outer = max(boxs_filter1_tmp[i].size.width, boxs_filter1_tmp[i].size.height);
        if (isInROI(box_inner.center, outer_extream_points) && max_len_inner/max_len_outer > 0.3 && max_len_inner/max_len_outer < 0.7)
        {
            // add offset
            for (int c = 0;  c < (int)roi_contours.size(); ++c)
            {
                for (int p = 0; p < (int)roi_contours[c].size(); ++p)
                {
                    roi_contours[c][p] = roi_contours[c][p] + Point((int)rect.x, (int)rect.y);
                }
            }
            contours_filter2.push_back(roi_contours[roi_max_idx]);


            boxs_filter2.push_back(minAreaRect(roi_contours[roi_max_idx]));
            contours_filter1.push_back(contours[i]);
            boxs_filter1.push_back(minAreaRect(contours[i]));
        }
    }
    if ((int)contours_filter1.size() == 0)
    {
        return false;
    }

    // If there are multiple nested boxes, choose the largest pair
    int max_idx = 0;
    float max_area = boxs_filter2[0].size.area();
    for (int i = 1; i < (int)boxs_filter2.size(); ++i)
    {
        float area = boxs_filter2[i].size.area();
        if (area > max_area)
        {
            max_idx = i;
            max_area = area;
        }
    }
    this->box = boxs_filter1[max_idx];
    tarContour_outter = contours_filter1[max_idx];
    tarContour_inner = contours_filter2[max_idx];
    return true;
}
double xyVision::GetTarget::computeRotatedRect(const vector<Point>& contour, cv::Size sz, RotatedRect box, Mat& bi)
{
    // use outter box if timeModel == DAYTIME, use inner box if timeModel == NIGHT
    Size2f inner_size = Size2f(box.size.width / 2, box.size.height / 2);
    RotatedRect inner_box = RotatedRect(box.center, inner_size, box.angle);

    Point2f vertices[4];
    if (timeModel == DAYTIME)
    {
        box.points(vertices);
    }
    else if (timeModel == NIGHT)
    {
        inner_box.points(vertices);
    }
    Point points[4];
    for (int i = 0; i < 4; ++i)
    {
        points[i] = Point((int)vertices[i].x, (int)vertices[i].y);
    }

    Mat tmp = Mat::zeros(sz, CV_8U);
    cv::fillConvexPoly(tmp, points, 4, Scalar(1));

    Mat boxIdx;
    findNonZero(tmp, boxIdx);
    double n_pixels = (double)boxIdx.total();
    double n_nonZero = 0;
    for (unsigned int i = 0; i < boxIdx.total(); ++i)
    {
        Point _p = boxIdx.at<Point>(i);
        if (bi.at<unsigned char>(_p.y, _p.x) > 0)
        {
            n_nonZero = n_nonZero + 1;
        }
    }
    return n_nonZero / n_pixels;
}


void xyVision::GetTarget::locating(vector<Point> & tarContour)
{
    //float scaleFactor = this->proInfo.scale;
    float boardWidth = this->targetInfo.targetWidth;
    Matx33f _K = this->cameraInfo.newCameraMatrix;

    RotatedRect _box = minAreaRect(Mat(tarContour));
    this->box = _box;

    float aveLen;
    // when box center is on boundary, choose the max length
    //if (this->frameCounterTarget == 0 || this->lastLoc.z < 500)
    if (_box.center.x - _box.size.width< 30 || _box.center.x + _box.size.width > cameraInfo.newSize.width - 30 ||
        _box.center.y - _box.size.height < 30 || _box.center.y + _box.size.height > cameraInfo.newSize.height - 30)
    {
        aveLen = std::max(_box.size.width, _box.size.height);
    }
    else
    {
        aveLen = std::min(_box.size.width, _box.size.height);
    }
    //float aveLen = (_box.size.width + _box.size.height) / 2;
    Point2f vtx[4];
    _box.points(vtx);
    Point2f center2d = _box.center;
    Point3f center3d = Point3f(center2d.x, center2d.y, 1.0f);
    Point3f pointInPlane = _K.inv() * center3d;
    this->currentLoc = pointInPlane * _K(0,0) * (boardWidth / aveLen);
    this->lastLoc = this->currentLoc;
}

void xyVision::GetTarget::locating_verbose(vector<vector<Point> > & tarContours)
{
    float boardWidth = this->targetInfo.targetWidth;
    Matx33f _K = this->cameraInfo.newCameraMatrix;
    this->allLoc.clear();
    for (int i = 0; i < (int)tarContours.size(); ++i)
    {
        float aveLen;
        RotatedRect _box = minAreaRect(Mat(tarContours[i]));
        if (_box.center.x - _box.size.width< 30 || _box.center.x + _box.size.width > cameraInfo.newSize.width - 30 ||
            _box.center.y - _box.size.height < 30 || _box.center.y + _box.size.height > cameraInfo.newSize.height - 30)
        {
            aveLen = std::max(_box.size.width, _box.size.height);
        }
        else
        {
            aveLen = std::min(_box.size.width, _box.size.height);
        }
        Point2f center2d = _box.center;
        Point3f center3d = Point3f(center2d.x, center2d.y, 1.0f);
        Point3f pointInPlane = _K.inv() * center3d;
        Point3f location = pointInPlane * _K(0,0) * (boardWidth / aveLen);
        this->allLoc.push_back(location);
    }
}


void xyVision::GetTarget::locating_tar_ver2(vector<Point> & tarContourInner, vector<Point>& tarContourOuter)
{
    //float scaleFactor = this->proInfo.scale;
    float outerWidth = this->targetInfo.targetWidth;
    float innerWidth = this->targetInfo.targetWidthInner;
    Matx33f _K = this->cameraInfo.newCameraMatrix;

    RotatedRect outerBox = minAreaRect(Mat(tarContourOuter));
    RotatedRect innerBox = minAreaRect(Mat(tarContourInner));

    // use inner box when on the boundary
    float aveLen;
    bool useInnerBox;
    if (isBoxOnBoundary(innerBox, this->cameraInfo.newSize, 30))
    {
        aveLen = std::max(innerBox.size.width, innerBox.size.height);
        useInnerBox = true;
    }
    else
    {
        aveLen = std::max(outerBox.size.width, outerBox.size.height);
        useInnerBox = false;
    }
    if (useInnerBox)
    {
        Point2f center2d = innerBox.center;
        Point3f center3d = Point3f(center2d.x, center2d.y, 1.0f);
        Point3f pointInPlane = _K.inv() * center3d;
        this->currentLoc = pointInPlane * _K(0,0) * (innerWidth / aveLen);
    }
    else
    {
        Point2f center2d = outerBox.center;
        Point3f center3d = Point3f(center2d.x, center2d.y, 1.0f);
        Point3f pointInPlane = _K.inv() * center3d;
        this->currentLoc = pointInPlane * _K(0,0) * (outerWidth / aveLen);
    }
    this->lastLoc = this->currentLoc;
}
void xyVision::GetTarget::runOneFrame()
{
    Mat _img = this->currentImg.clone();

    // convert to pinhole camera
    adjustImg(_img);

    if (this->frameCounter % 3 == 0)
    {
        this->readVersion(this->versionFileName);
    }


    if (this->targetInfo.targetVersion == 3)
    {
        vector<int> markerIds;
        vector< vector<Point2f> > markerCorners;
        aruco::detectMarkers(_img, this->targetInfo.board.dictionary, markerCorners, markerIds);
        if (markerIds.size() > 0)
        {

            cv::Mat rvec, tvec;
            this->isDetected = (aruco::estimatePoseBoard(markerCorners, markerIds,
                this->targetInfo.board, this->cameraInfo.newCameraMatrix,
                cv::Mat::zeros(1, 5, CV_32F), rvec, tvec) > 0);
            this->currentLoc = Point3f(tvec);
            this->allLoc.clear();
            this->allLoc.push_back(this->currentLoc);
            this->lastLoc = this->currentLoc;
        }
        else
        {
            this->allLoc.clear();
            this->isDetected = false;
        }
        return;
    }

    Mat bi;
    vector<Point> tarContour, tarContourInner, tarContourOuter;
    vector<vector<Point> > tarContours;
    bool detected = true;

    if (targetInfo.targetVersion == 1 || (targetInfo.targetVersion == 2 && timeModel == NIGHT))
    {
        if (timeModel == DAYTIME)
        {
            this->binarizeTarget_LAB(_img, bi);
        }
        else if (timeModel == NIGHT)
        {
            this->binarizeTarget_night(_img, bi);

        }
        //if (outputModel == OUTPUT_FILTER)
        //{
       detected = this->contourDetect(bi, tarContour, tarContours);
        //}
        //else if (outputModel == OUTPUT_VERBOSE)
        //{
        //    detected = this->contourDetect_verbose(bi, tarContours);
        //}
    }
    else if (targetInfo.targetVersion == 2)
    {
        detected = this->contourDetect_day_tar_ver2(_img, tarContourOuter, tarContourInner);
    }

    if (!detected)
    {
        isDetected = false;
        return;
    }
    else
    {
        isDetected = true;
    }

    if (targetInfo.targetVersion == 1)
    {
        locating(tarContour);
        locating_verbose(tarContours);   
    }
    else if (targetInfo.targetVersion == 2)
    {
        locating_tar_ver2(tarContourInner, tarContourOuter);
    }
}

void xyVision::GetTarget::runOneFrame_gpu()
{

    //clock_t start,finish;
    //double totaltime;

    //start = clock();

    Mat _img = currentImg.clone();
    gpu::GpuMat _img_gpu;
    _img_gpu.upload(_img);
    adjustImg_gpu(_img_gpu);

    //finish = clock();
    //totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
    //time1 = time1 + totaltime;

    vector<Point> tarContour;
    vector<vector<Point> > tarContours;
    bool detected;

    //start = clock();

    gpu::GpuMat bi_gpu;
    binarizeTarget_gpu(_img_gpu, bi_gpu);

    //finish = clock();
    //totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
    //time2 = time2 + totaltime;

    //start = clock();

    Mat bi;
    bi_gpu.download(bi);

    detected = this->contourDetect(bi, tarContour, tarContours);

    if (!detected)
    {
        isDetected = false;
        return;
    }
    else
    {
        isDetected = true;
    }
    locating(tarContour);
}

void xyVision::GetTarget::clearStates()
{
    frameCounter = 0;
    frameCounterTarget = 0;
    isDetected = true;
    this->useGPU = false;
    this->targetInfo.targetVersion = this->oriTargetVersion;
}

void xyVision::GetTarget::setTimeModel(int model)
{
    this->timeModel = model;
}
//void xyVision::GetTarget::runOneFrame()
//{
//	Matx33f _K = this->cameraInfo.cameraMatrix;
//	float scaleFactor = this->proInfo.scale;
//	float boardWidth = this->targetInfo.targetWidth;
//	Mat img = this->currentImg;
//	Mat _img = img.clone();
//
//	clock_t start,finish;
//	double totaltime;
//	start=clock();
//	//gpu::GpuMat _imgGpu;
//	//cout << gpu::getCudaEnabledDeviceCount() << endl;
//	if (scaleFactor != 1.0f)
//	{
//		Size sz = img.size();
//		_K = _K * scaleFactor;
//		_K(2, 2) = 1.0f;
//
//
//		//_imgGpu.upload(_img);
//		//gpu::resize(_imgGpu, _imgGpu, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
//		resize(_img, _img, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
//	}
//	//_imgGpu.download(_img);
//	finish = clock();
//	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
//	//cout << "time for resize " << totaltime << endl;
//
//	start=clock();
//	this->adjustImg(_img);
//	finish = clock();
//	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
//	//cout << "time for adjust image " << totaltime << endl;
//	Mat bi;
//	vector<Point> tarContour;
//	bool detected;
//
//	start = clock();
//	detected = this->binarizeTarget(_img, bi, tarContour);
//	finish = clock();
//	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
//	//cout << "time for binaryTarget " << totaltime << endl;
//
//	if (!detected)
//	{
//		isDetected = false;
//		return;
//	}
//	else
//	{
//		isDetected = true;
//	}
//
//	start = clock();
//	RotatedRect box = minAreaRect(Mat(tarContour));
//	finish = clock();
//	totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
//	//cout << "time for locating " << totaltime << endl;
//
//	//float aveLen = std::max(box.size.width, box.size.height);
//	float aveLen = (box.size.width + box.size.height) / 2;
//	Point2f vtx[4];
//	box.points(vtx);
//	Point2f center2d = box.center;
//	Point3f center3d = Point3f(center2d.x, center2d.y, 1.0f);
//	Point3f pointInPlane = _K.inv() * center3d;
//	this->currentLoc = pointInPlane * _K(0,0) * (boardWidth / aveLen);
//}
int xyVision::GetTarget::readVersion(const string& fileName)
{
    ifstream reader;
    reader.open(fileName.c_str(), ios::in);
    string line;
    getline(reader, line);
    int version;
    sscanf(line.c_str(), "%d", &version);
    if (version == 1 || version == 2 || version == 3)
    {
        this->targetInfo.targetVersion = version;
    }
    else
    {
        this->targetInfo.targetVersion = this->oriTargetVersion;
    }
    return version;
}