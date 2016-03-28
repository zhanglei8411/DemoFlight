#ifndef __GET_TARGET_LOCATION__
#define __GET_TARGET_LOCATION__

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/gpu/gpumat.hpp"
#include "opencv2/video/tracking.hpp"
#include "aruco.hpp"
#include <string>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

/** @brief class for locating target plane.
    Sample usage:
    GetTarget sample("config.ini");
    Mat image = cv::imread("test.bmp");
    sample << image;
    cout << Mat(sample.getCurrentLoc());
*/

namespace xyVision{
enum{
    DAYTIME,
    NIGHT
};
enum{
    OUTPUT_FILTER,
    OUTPUT_VERBOSE
};
struct cameraParams
{
    Matx33f cameraMatrix;
    Matx14f distortCoeff;
	int cameraType;
	Size imageSize;
	Size newSize;
	Matx33f newCameraMatrix;
	Mat map1, map2;
};

struct targetInformation
{
    float targetWidth;
    float targetWidthInner;
    int targetVersion;
    aruco::Board board;  
};

struct processingInformation
{
	float scale;
};

class GetTarget
{
public:

    GetTarget(string configName);

    /** @brief set camera parameters.
        @param configName Input filename of config file
    */
    void setCameraParams(std::string configName);

    /** @brief give one frame to algorithm
        @param image Input image
    */
    GetTarget& operator<<(const Mat& image);

    /** @brief get target location of current frame
    */
    Point3f getCurrentLoc();

    vector<Point3f> getAllLoc();
	/** @brief clear current states
	*/
	void clearStates();

    /** @brief set time model
        @param model xyVision::DAYTIME or xyVision::NIGHT
    */
    void setTimeModel(int model);

	int frameCounter;
	int frameCounterTarget;

	bool isDetected;

	bool useGPU;

	double time1, time2, time3;

	RotatedRect box;

	KalmanFilter KF;

	cv::Mat rectified;

	double lastArea;

	cv::Point3f lastLoc;

	vector<int> mapping;

    int timeModel;

    targetInformation targetInfo;

    cameraParams cameraInfo;

    int oriTargetVersion;

    
private:
    Point3f currentLoc;
    vector<Point3f> allLoc;
    Mat currentImg, previousImg;
    
	processingInformation proInfo;
    string configFileName;
    string versionFileName;

    void adjustImg(Mat & img);
	void adjustImg_gpu(gpu::GpuMat& img);
    void binarizeTarget(const Mat & img, Mat & bi);
	void binarizeTarget_HSV(const Mat& img, Mat & bi);
	void binarizeTarget_LAB(const Mat& img, Mat & bi);
	void binarizeTarget_gpu(const gpu::GpuMat img, gpu::GpuMat & bi);
    void binarizeTarget_night(const Mat& img, Mat & bi);
    
    void imadjust(const Mat1b& src, Mat1b& dst, int tol = 1, Vec2i in = Vec2i(0, 255), Vec2i out = Vec2i(0, 255));
	//void imadjust_gpu(const gpu::GpuMat& src, gpu::GpuMat& des, int tol = 1, Vec2i in = Vec2i(0, 255), Vec2i out = Vec2i(0, 255));
	void imadjust_mapping(const Mat& src, Mat& dst, vector<int> mapping);
	void locating(vector<Point> & tarContour);
    void locating_verbose(vector<vector<Point> > & tarContours);
    void locating_tar_ver2(vector<Point> & tarContour_inner, vector<Point>& tarContour_outer);
	bool contourDetect(Mat& bi, vector<Point> & tarContour, vector<vector<Point> > & tarContours);
   // bool contourDetect_verbose(Mat& bi, vector<vector<Point> > & tarContours);
    bool contourDetect_day_tar_ver2(const Mat& img, vector<Point>& tarContour_outter, vector<Point>& tarContour_inner);
	void runOneFrame();
	void runOneFrame_gpu();
	double computeRotatedRect(const vector<Point>& contour, cv::Size sz, RotatedRect box, Mat& bi);
	cv::Mat equalizeIntensity(const Mat& inputImage);
    int readVersion(const string& fileName);
};

}

#endif