#ifndef __GET_TARGET_LOCATION__
#define __GET_TARGET_LOCATION__

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <iostream>

using namespace cv;
using namespace std;

/** @brief class for locating target plane.
    Sample usage:
    xyVision::GetTarget sample("config.ini");
    Mat image = cv::imread("test.bmp");
    sample << image;
    cout << Mat(sample.getCurrentLoc());
*/

namespace xyVision{

struct cameraParams
{
    Matx33f cameraMatrix;
    Matx14f distortCoeff;
};

struct targetInformation
{
    float targetWidth;
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

	int frameCounter;
private:
    Point3f currentLoc;
    Mat currentImg, previousImg;
    targetInformation targetInfo;
	cameraParams cameraInfo;
	processingInformation proInfo;
    string configFileName;

    void adjustImg(Mat & img);
    void binarizeTarget(const Mat & img, Mat & bi, vector<Point> & tarContour);
    void imadjust(const Mat1b& src, Mat1b& dst, int tol = 1, Vec2i in = Vec2i(0, 255), Vec2i out = Vec2i(0, 255));
	void runOneFrame();
};

}

#endif