#include "opencv.hpp"
#include "stdio.h"
#include <time.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "getTargetLoc.hpp"

using namespace cv;
using namespace std;


int main()
{
	double totaltime;
	clock_t start,finish;

	CvCapture* capture=NULL;
	IplImage *ipllmage;
	
	//capture = cvCreateCameraCapture(200);	//200表示CV_CAP_V4L/CV_CAP_V4L2

	xyVision::GetTarget sample("config.ini");
	capture = cvCreateCameraCapture(0);		//0表示CV_CAP_ANY，在只有一个摄像头的时候可以使用这个
	while(1)
	{
		start=clock();
		
		ipllmage = cvQueryFrame( capture );	//从摄像头中抓取并返回一帧
		if( !ipllmage ) break;

		//Mat mat_frame(ipllmage, 1);
		Mat mat_frame = ipllmage;			//ipllmage --> Mat
		imshow("usb cam", mat_frame);

		sample << mat_frame;
		finish = clock();
		
		totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
		cout << "Target Location" << endl;
		cout << Mat(sample.getCurrentLoc())<<endl;
		cout << "Average running time for a frame " << totaltime << endl;
		
#if 0
		cvNamedWindow("usbcam", 0);			//创建一个窗口用来显示图像, 0表示用户可以手动调节窗口大小，且显示的图像尺寸随之变化
		cvShowImage( "usbcam", ipllmage );	//显示图像
#endif

		//等待20ms在显示下一帧视频
		cvWaitKey(20);
	}


	//释放摄像头
	cvReleaseCapture(&capture);


	return 0;
}




