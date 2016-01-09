#include "image_identify.h"
#include "thread_common_op.h"
#include <semaphore.h>

using namespace cv;
using namespace std;

Offset offset_data;

pthread_mutex_t capture_on_off_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t offset_msg_lock = PTHREAD_MUTEX_INITIALIZER;
int capture_on_flag = 0;
sem_t capture_start_sem;

sem_t image_get_sem;
sem_t image_handle_sem;
sem_t offset_get_sem;

int tell_external_offset_is_available(void)
{
	return sem_post(&offset_get_sem);		//if error return -1
}

int check_offset_data_if_available(void)
{
	int i = 0;
	while(sem_trywait(&offset_get_sem) == 0)
	{
		i++;
	}
	if(i != 0)
		return 0;							//available
	else
		return -1;
}

void set_offset_data(Point3f _point)
{
	pthread_mutex_lock(&offset_msg_lock);
	offset_data.x = _point.x;
	offset_data.y = _point.y;
	offset_data.z = _point.z;
	pthread_mutex_unlock(&offset_msg_lock);
}

Offset get_offset_data(void)
{
	Offset ret;
	pthread_mutex_lock(&offset_msg_lock);
	ret = offset_data;
	pthread_mutex_unlock(&offset_msg_lock);

	return ret;
}

int XY_Get_Offset_Data(Offset *_data)
{
	if(check_offset_data_if_available() == -1)
	{
		return -1;
	}

	*_data = get_offset_data();
	return 0;
}



void set_capture_on_flag(int _val)
{
	_val = _val > 1 ? 1 : _val;
	pthread_mutex_lock(&capture_on_off_lock);
	capture_on_flag = _val;
	pthread_mutex_unlock(&capture_on_off_lock);
}

void clear_capture_on_flag(void)
{
	set_capture_on_flag(0);
}

int get_capture_on_flag(void)
{
	int ret;
	pthread_mutex_lock(&capture_on_off_lock);
	ret = capture_on_flag;
	pthread_mutex_unlock(&capture_on_off_lock);

	return ret;
}

int if_capture_on(void)
{
	return get_capture_on_flag();
}

void wait_capture_on(void)
{
	sem_wait(&capture_start_sem);
}


int XY_Start_Capture(void)
{
	set_capture_on_flag(1);
	sem_post(&capture_start_sem);

	return 0;
}

int XY_Stop_Capture(void)
{
	set_capture_on_flag(0);

	return 0;
}

stringstream stream;
string dirname = "/mnt/sdcard/image/";
string total_filename;
float cur_height;
int save_image_into_sdcard(IplImage *_image)
{
	XY_Pro_Get_Pos_Height(&cur_height);
	//printf("%f\n", cur_height);
	
	stream.str("");
	stream << cur_height;
	total_filename = dirname + "H" + stream.str() + ".jpg";
	const char *p = total_filename.c_str();
	cvSaveImage(p, _image);
	
}

IplImage *image;
//Mat img;
static void *capture_thread_func(void * arg)
{	
	//thread_binding_cpu(NULL, CAPTURE_JOB_CPU);

	CvCapture* capture = NULL;
	capture = cvCreateCameraCapture(0);
	
	int time_ = 0;
	char *p = NULL;
	
	while(1)
	{	
		if(if_capture_on())
		{
			sem_wait(&image_handle_sem);	//除第一次外，等待image_identify_thread_func处理完图像
			image = cvQueryFrame( capture );
			if( !image )
			{ 
				printf("No image!\n");
				sleep(2);
				sem_post(&image_handle_sem);
				
				continue;
			}
			time_++;
			if(time_ == 5)
			{
				save_image_into_sdcard(image);
				time_ = 0;
			}
			sem_post(&image_get_sem);		
			usleep(100000);			//100ms时CPU占用率为7.7%左右
		}
		else
		{
			wait_capture_on();
		}
	}
	pthread_exit(NULL);
}


static void *image_identify_thread_func(void * arg)
{
	//thread_binding_cpu(NULL, IMAGE_JOB_CPU);
	
	xyVision::GetTarget sample("config.ini");
	while(1)
	{	
		sem_wait(&image_get_sem);		//等待capture获取到图像
#if 0
		Mat mat_frame = image;			//IplImage --> Mat

		sample << mat_frame;			//identify
		if(!sample.isDetected )
			goto pre_restart;

		cout << sample.getCurrentLoc() << endl;

		set_offset_data(sample.getCurrentLoc());
		tell_external_offset_is_available();
#endif
pre_restart:
		sem_post(&image_handle_sem);
	}
	pthread_exit(NULL);
}

int setup_sem(void)
{
	int ret;

	ret = sem_init(&capture_start_sem, 0, 0);
	ret += sem_init(&image_get_sem, 0, 0);
	ret += sem_init(&image_handle_sem, 0, 1);
	ret += sem_init(&offset_get_sem, 0, 0);
	return ret;
}

int ctreate_capture_and_identify_thread(void)
{
	int ret;

	ret = setup_sem();
	if(ret != 0)
	{
		printf("semaphore initialization failed.\n");
		return -1;
	}

	if(XY_Create_Thread(capture_thread_func, NULL, THREADS_CAPTURE, -1, SCHED_RR, 92) < 0)
	{
		printf("Create Capture Thread Error.\n");
		return -1;
	}

	if(XY_Create_Thread(image_identify_thread_func, NULL, THREADS_IDENTIFY, -1, SCHED_RR, 91) < 0)
	{
		printf("Create Image Identify Thread Error.\n");
		return -1;
	}

	return 0;

#if 0
	pthread_t tid;
	ret = pthread_create(&tid, 0,capture_thread_func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	ret = pthread_create(&tid, 0,image_identify_thread_func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
#endif 
}

int XY_Capture_Setup(void)
{
	return ctreate_capture_and_identify_thread();
}

