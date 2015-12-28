#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <iostream>
#include "getTargetLoc.hpp"
#include "inifile.h"

using namespace cv;
using namespace std;

#define  BUF_SIZE 256 

xyVision::GetTarget::GetTarget(string configName)
{
	this->setCameraParams(configName);
	frameCounter = 0;
}

void xyVision::GetTarget::setCameraParams(string configFileName)
{
	// read camera parameters
	this->configFileName = configFileName;
	const char * section = "cameraConfig";
	const char * key = "fx";
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

	Matx33f _cameraMat(fx, 0, u0, 0, fy, v0, 0, 0, 1);
	Matx14f _dis(k1, k2, k3, k4);
	this->cameraInfo.cameraMatrix = _cameraMat;
	this->cameraInfo.distortCoeff = _dis;
	
	// read target information
	section = "targetConfig";
	key = "width";
	float width = read_profile_float(section, key, 0, configFileName.c_str());
	this->targetInfo.targetWidth = width;

	// read processing information
	section = "processingConfig";
	key = "scale";
	float scale = read_profile_float(section, key, 0, configFileName.c_str());
	this->proInfo.scale = scale;
}

xyVision::GetTarget& xyVision::GetTarget::operator<<(const Mat& image)
{
	if (this->frameCounter > 0)
	{
		this->previousImg = this->currentImg.clone();
	}
	this->currentImg = image.clone();

	this->runOneFrame();
	this->frameCounter ++;
	return *this;
}

Point3f xyVision::GetTarget::getCurrentLoc()
{
	return this->currentLoc;
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
		in[0] = distance(cum.begin(), lower_bound(cum.begin(), cum.end(), low_bound));
		in[1] = distance(cum.begin(), lower_bound(cum.begin(), cum.end(), upp_bound));
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
	vector<Mat> chs;
	split(img, chs);
	Mat1b out;
	// b g r
	imadjust(chs[2], out);
	chs[2] = out;
	merge(chs, img);
}

void xyVision::GetTarget::binarizeTarget(const Mat & img, Mat & bi, vector<Point> & tarContour)
{
	// use tarContour to detect target when computation resource is limited
	CV_Assert(img.channels() == 3);
	std::vector<Mat> chs;
	//Mat img_f;
	//img.convertTo(img_f, CV_32FC3);
	split(img, chs);
	Mat img_t = chs[2] - chs[0] - chs[1];
	cv::threshold(img_t, bi, 10, 255, THRESH_BINARY);
	bi.convertTo(bi, CV_8UC1);

	// improve binary image
	Mat kernel = cv::Mat::ones(3, 3, CV_8UC1);
	dilate(bi, bi, kernel);
	erode(bi, bi, kernel);
	// remove small object
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(bi.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	int maxIdx = 0;
	int maxAera = (int)contourArea(contours[0]);
	for (int i = 1; i < (int)contours.size(); ++i)
	{
		int conAera_i = (int)contourArea(contours[i]);
		if (conAera_i > maxAera)
		{
			maxAera = conAera_i;
			maxIdx = i;
		}
	}
	for (int i = 0; i < (int)contours.size(); ++i)
	{
		if (i != maxIdx)
		{
			drawContours(bi, contours, i, cv::Scalar(0), CV_FILLED, 8);
		}
	}
	tarContour = contours[maxIdx];
}

void xyVision::GetTarget::runOneFrame()
{
	Matx33f _K = this->cameraInfo.cameraMatrix;
	float scaleFactor = this->proInfo.scale;
	float boardWidth = this->targetInfo.targetWidth;
	Mat img = this->currentImg;
	Mat _img = img.clone();
	if (scaleFactor != 1)
	{
		Size sz = img.size();
		_K = _K * scaleFactor;
		_K(2, 2) = 1.0f;
		resize(_img, _img, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
	}
	this->adjustImg(_img);
	Mat bi;
	vector<Point> tarContour;
	this->binarizeTarget(_img, bi, tarContour);
	RotatedRect box = minAreaRect(Mat(tarContour));
	float aveLen = (box.size.width + box.size.height) / 2;
	Point2f vtx[4];
	box.points(vtx);
	Point2f center2d = box.center;
	Point3f center3d = Point3f(center2d.x, center2d.y, 1.0f);
	Point3f pointInPlane = _K.inv() * center3d;
	this->currentLoc = pointInPlane * _K(0,0) * (boardWidth / aveLen);
}