#include "image_identify.h"

using namespace cv;
using namespace std;

Offset mark_offset;
pthread_mutex_t offset_msg_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t refresh_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t refresh_cond = PTHREAD_COND_INITIALIZER;
volatile int refresh_flag = 0;

void store_offset3f(Offset *_offset, Point3f _point)
{
	_offset->x = _point.x;
	_offset->y = _point.y;
	_offset->z = _point.z;
}

Offset get_stored_offset_block(void)
{
	Offset _offset = {0};

	pthread_mutex_lock(&offset_msg_lock);
	_offset = mark_offset;
	pthread_mutex_unlock(&offset_msg_lock);
	return _offset;
}

void set_refresh_flag(void)
{
	pthread_mutex_lock(&refresh_mutex);
	refresh_flag = 1;
	pthread_mutex_unlock(&refresh_mutex);
}

void clear_refresh_flag(void)
{
	pthread_mutex_lock(&refresh_mutex);
	refresh_flag = 0;
	pthread_mutex_unlock(&refresh_mutex);
}

int read_refresh_flag(void)
{
	int t;
	pthread_mutex_lock(&refresh_mutex);
	t = refresh_flag;
	pthread_mutex_unlock(&refresh_mutex);

	return t;
}


static void *capture_and_identify_thread_func(void * arg)
{
	CvCapture* capture=NULL;
	IplImage *ipllmage;
	
	double totaltime;
	clock_t start,finish;	

	xyVision::GetTarget sample("config.ini");
	capture = cvCreateCameraCapture(0);
	
	while(1)
	{	
#if 0
		start=clock();
#endif
		
		ipllmage = cvQueryFrame( capture );	//从摄像头中抓取并返回一帧
		if( !ipllmage ) 
			continue;
		Mat mat_frame = ipllmage;			//ipllmage --> Mat

		sample << mat_frame;
		if(!sample.isDetected )
			continue;
#if 0
		finish = clock();
		totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
		cout << "Average running time for a frame " << totaltime << endl;
#endif

		//cout << sample.getCurrentLoc() << endl;
		pthread_mutex_lock(&offset_msg_lock);
		store_offset3f(&mark_offset, sample.getCurrentLoc());
		pthread_mutex_unlock(&offset_msg_lock);
		
		set_refresh_flag();

#if 0
		printf("%f %f %f\n", mark_offset.x, mark_offset.y, mark_offset.z);
#endif

		usleep(100000);
	}
	pthread_exit(NULL);
}


int XY_Ctreate_Capture_and_Identify_Thread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0,capture_and_identify_thread_func,NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}

