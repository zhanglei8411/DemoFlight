#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/gpu/gpumat.hpp"
#include "opencv2/video/tracking.hpp"
#include "fisheye.hpp"
#include <string>
#include <iostream>
#include <stdio.h>
#include "getTargetLoc.hpp"
#include "inifile.h"
#include "time.h"

using namespace cv;
using namespace std;

#define  BUF_SIZE 256 

xyVision::GetTarget::GetTarget(string configName)
{
	this->setCameraParams(configName);
	frameCounter = 0;
	isDetected = true;
	this->useGPU = false;
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
		cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraInfo.cameraMatrix, cameraInfo.distortCoeff,
			cameraInfo.imageSize, cv::Matx33d::eye(), cameraInfo.newCameraMatrix, 0.8, cameraInfo.newSize, 1);
		cv::fisheye::initUndistortRectifyMap(cameraInfo.cameraMatrix, cameraInfo.distortCoeff, 
			cv::Matx33d::eye(), cameraInfo.newCameraMatrix, cameraInfo.newSize, CV_16SC2, cameraInfo.map1, cameraInfo.map2);
	}

	// initialize Kalman fileter
	KF = KalmanFilter(6, 3, 0);
	KF.transitionMatrix = *(Mat_<float>(6,6) << 1,0,0,1,0,0,   0,1,0,0,1,0,  0,0,1,0,0,1,  0,0,0,1,0,0,  0,0,0,0,1,0,  0,0,0,0,0,1);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(3));
	setIdentity(KF.errorCovPost, Scalar::all(.1));

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
	this->targetInfo.targetWidth = width;

	// read processing information
	section = "processingConfig";
	key = "scale";
	float scale = read_profile_float(section, key, 0, configFileName.c_str());
	this->proInfo.scale = scale;

	this->cameraInfo.newSize = Size(int(imageWidth*scale), int(imageHeight*scale));
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
		if (this->frameCounter == 0)
		{
			
			KF.statePre = Mat(Matx16f(_point.x, _point.y, _point.z, 0.f, 0.f, 0.f));
		}
		else if (this->frameCounter > 0)
		{
			KF.predict();
			Mat estimate = KF.correct(Mat(_point));
			this->currentLoc = Point3f(estimate.at<float>(0), estimate.at<float>(1), estimate.at<float>(2));
		}
		this->frameCounter ++;
	}
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
	//CV_Assert(img.channels() == 3);
	if (img.channels() != 3)
	{	
		printf("img channel is not 3 \n");
		return ;
	}
	
	Size sz = img.size();
	float scaleFactor = this->proInfo.scale;
	if (cameraInfo.cameraType == 1)
	{
		cv::resize(img, img, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
	}
	else
	{
		cv::remap(img, img, cameraInfo.map1, cameraInfo.map2, INTER_LINEAR, BORDER_CONSTANT);
		this->rectified = img.clone();
	}

	vector<Mat> chs;
	split(img, chs);
	Mat1b out;
	// b g r
	imadjust(chs[2], out);
	//cv::equalizeHist(chs[2], out);
	chs[2] = out;
	merge(chs, img);
}

void xyVision::GetTarget::adjustImg_gpu(gpu::GpuMat& img)
{
	CV_Assert(img.channels() == 3);

	Size sz = img.size();
	float scaleFactor = this->proInfo.scale;
	gpu::GpuMat img2;
	gpu::resize(img, img2, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)));
	img = img2.clone();

	int a = gpu::getCudaEnabledDeviceCount();
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

void xyVision::GetTarget::binarizeTarget(const Mat & img, Mat & bi)
{
	CV_Assert(img.channels() == 3);
	
	std::vector<Mat> chs;
	
	Mat img2 = img.clone();
	//img2.convertTo(img2, CV_32FC3);
	split(img2, chs);
	Mat img_t = chs[2] - chs[0] - chs[1];
	cv::threshold(img_t, bi, 10, 255, THRESH_BINARY);
	bi.convertTo(bi, CV_8UC1);
	
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

	gpu::threshold(img_t, bi, 10, 255, THRESH_BINARY);

	bi.convertTo(bi, CV_8UC1);

	// resize binary image
	//Size sz = img2.size();
	//float scaleFactor = this->proInfo.scale;
	//gpu::GpuMat bi2, bi3, bi4;
	//gpu::resize(bi, bi2, Size(int(sz.width*scaleFactor), int(sz.height*scaleFactor)), scaleFactor, scaleFactor, cv::INTER_NEAREST);
	//bi = bi2.clone();
	// improve binary image
	//Mat kernel = cv::Mat::ones(3, 3, CV_8UC1);
	//gpu::dilate(bi2, bi3, kernel);
	//gpu::erode(bi3, bi4, kernel);
	//bi = bi4.clone();
}

bool xyVision::GetTarget::contourDetect(Mat& bi, vector<Point> & tarContour)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Size sz = bi.size();

	//clock_t start,finish;
	//double totaltime;

	findContours(bi.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	//cout << "time for finding Contours " << totaltime << endl;
	// no contours
	if ((int)contours.size() == 0)
		return false;

	// filter areas that are not square
	//vector<vector<Point> > contours_filter;
	//for (int i = 0; i < (int)contours.size(); ++i)
	//{
	//	//if (i == (int)contours.size() - 1)
	//	//{
	//	//	cout << " " << endl;
	//	//}
	//	RotatedRect box = minAreaRect(Mat(contours[i]));
	//	float height = float(box.size.height);
	//	float width = float(box.size.width);
	//	if (height / width < 3 && width / height < 3)
	//	{
	//		contours_filter.push_back(contours[i]);
	//	}
	//}
	//contours = contours_filter;

	if ((int)contours.size() == 0)
	{
		return false;
	}
	// choose the largest one
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
	if (maxAera < (sz.width/50.0f*sz.height/50.0f))
		return false;
	// set zeros for other contours
	for (int i = 0; i < (int)contours.size(); ++i)
	{
		if (i != maxIdx)
		{
			drawContours(bi, contours, i, cv::Scalar(0), CV_FILLED, 8);
			//drawContours(img_t, contours, i, cv::Scalar(0), CV_FILLED, 8);
		}
	}
	tarContour = contours[maxIdx];
	return true;
}
void xyVision::GetTarget::locating(vector<Point> & tarContour)
{
	float scaleFactor = this->proInfo.scale;
	float boardWidth = this->targetInfo.targetWidth;
	Matx33f _K = this->cameraInfo.newCameraMatrix;
	//if (scaleFactor != 1.0)
	//{
	//	Size sz = currentImg.size();
	//	_K = _K * scaleFactor;
	//	_K(2, 2) = 1.0f;
	//}

	RotatedRect _box = minAreaRect(Mat(tarContour));
	this->box = _box;

	float aveLen = std::max(_box.size.width, _box.size.height);
	//float aveLen = (_box.size.width + _box.size.height) / 2;
	Point2f vtx[4];
	_box.points(vtx);
	Point2f center2d = _box.center;
	Point3f center3d = Point3f(center2d.x, center2d.y, 1.0f);
	Point3f pointInPlane = _K.inv() * center3d;
	this->currentLoc = pointInPlane * _K(0,0) * (boardWidth / aveLen);
}

void xyVision::GetTarget::runOneFrame()
{
	Mat _img = this->currentImg.clone();

	//clock_t start,finish;
	//double totaltime;
	//start = clock();

	adjustImg(_img);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time1 = time1 + totaltime;

	Mat bi;
	vector<Point> tarContour;
	bool detected;

	//start = clock();

	this->binarizeTarget(_img, bi);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time2 = time2 + totaltime;

	//start = clock();

	detected = this->contourDetect(bi, tarContour);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time3 = time3 + totaltime;

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

	detected = this->contourDetect(bi, tarContour);

	//finish = clock();
	//totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	//time3 = time3 + totaltime;

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