#include "wireless_debug.h"

static int debug_fd = -1;
static fd_set debug_fd_set;

struct debug_info debug_package = {0};
pthread_mutex_t debug_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t debug_cond = PTHREAD_COND_INITIALIZER;

pthread_mutex_t offset_debug_msg_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t zl_debug_msg_lock = PTHREAD_MUTEX_INITIALIZER;

extern pthread_mutex_t ultra_msg_lock;

volatile int offset_refresh_flag = 0;
volatile int zl_refresh_flag = 0;


int XY_Debug_Setup(void)
{
	int wireless_baudrate = 9600;

	return sdk_wireless_debug_setup(DEBUG_DEV_NAME, wireless_baudrate);
}

int XY_Debug_Easy_Send(char *buf, int len)
{
	return write(debug_fd, buf, len);
}

void XY_Debug_Get_Pos(position *pp, double _longti, double _lati, double _alti, float _height)
{
	pp->_longti = _longti;
	pp->_lati = _lati;
	pp->_alti = _alti;
	pp->_height= _height;
}


void XY_Debug_Get_UserCtrl(attitude *pa, float _roll, float _pitch, float _thr, float _yaw)
{
	pa->roll_or_x= _roll;
	pa->pitch_or_y= _pitch;
	pa->thr_z= _thr;
	pa->yaw = _yaw;
}

void XY_Debug_Get_Last_Dist(double *dst, double _last_distance)
{
	*dst = _last_distance;
}

void XY_Debug_Get_Ultrasonic_Data(float *result, float _ultra_data)
{
	*result = _ultra_data;
}

void XY_Debug_Get_Offset_Data(Offset *offset_dst, Offset offset_src)
{
	pthread_mutex_lock(&offset_debug_msg_lock);
	offset_dst->x = offset_src.x;
	offset_dst->y = offset_src.y;
	offset_dst->z = offset_src.z;
	offset_refresh_flag = 1;
	pthread_mutex_unlock(&offset_debug_msg_lock);
}

void XY_Debug_Get_Zhanglei_Data(Zhanglei *zl_dst, double _a, double _b)
{
	pthread_mutex_lock(&zl_debug_msg_lock);
	zl_dst->a = _a;
	zl_dst->b = _b;
	zl_refresh_flag = 1;
	pthread_mutex_unlock(&zl_debug_msg_lock);
}


int sdk_wireless_debug_setup(const char *device, int baudrate)
{
	if(uart_common_start(device, baudrate, &debug_fd, &debug_fd_set) < 0)
	{
		if(debug_fd > 0)
		{
			uart_common_close(&debug_fd);
		}
		return -1;
	}
#if 1
	if(create_wireless_debug_thread() < 0)
	{
		return -1;
	}
#endif

	return 0;
}

void sdk_handle_debug_info(struct debug_info info, char *str, int flag)
{
	switch(flag)
	{
		case 1:
#if 0
			sprintf(str, "\n[S](%.8lf, %.8lf) [Cur](%.8lf, %.8lf) [Height]%.8lf)", 	debug_package.start._longti,
																		debug_package.start._lati,  
																		debug_package.cur._longti,
																		debug_package.cur._lati,
																		debug_package.cur._height);
#endif
			sprintf(str, "\n[Height]%.8lf\n",debug_package.cur._height);
			break;
		case 2:
			sprintf(str, "\t[Ultra Data] %.6fm\n",	debug_package.ultrasonic_data);
			break;
			
		case 3:
			pthread_mutex_lock(&ultra_msg_lock);
			sprintf(str, "[Last Distance] %.8lfm\n",	debug_package.dist);
			pthread_mutex_unlock(&ultra_msg_lock);	
			break;
		case 4:
			pthread_mutex_lock(&offset_debug_msg_lock);
			offset_refresh_flag = 0;
			sprintf(str, "[Offset] %f, %f, %f\n",	debug_package.offset3f.x,
													debug_package.offset3f.y,
													debug_package.offset3f.z);
			pthread_mutex_unlock(&offset_debug_msg_lock);
			break;
		case 5:
			pthread_mutex_lock(&zl_debug_msg_lock);
			zl_refresh_flag = 0;
			sprintf(str, "[zl] %.8lf, %.8lf\n",	debug_package.zl.a,
													debug_package.zl.b);
			pthread_mutex_unlock(&zl_debug_msg_lock);
			break;
			
		default:
			break;
	}

}

int sdk_send_debug_info(char *str)
{
	return write(debug_fd, str, strlen((const char *)str));
}

static void *Wireless_Debug_Thread_Func(void * arg)
{
	char str[200];
	
	while(1)
	{	

		pthread_mutex_lock(&debug_mutex);
		pthread_cond_wait(&debug_cond , &debug_mutex);	
		pthread_mutex_unlock(&debug_mutex);
		sdk_handle_debug_info(debug_package, str, 1);
		sdk_send_debug_info(str);

#if 0	
		sdk_handle_debug_info(debug_package, str, 3);
		sdk_send_debug_info(str);	
		sdk_handle_debug_info(debug_package, str, 2);
		sdk_send_debug_info(str);
#endif

		if(zl_refresh_flag == 1)
		{
			sdk_handle_debug_info(debug_package, str, 5);
			sdk_send_debug_info(str);
		}
		if(offset_refresh_flag == 1)
		{
			sdk_handle_debug_info(debug_package, str, 4);
			sdk_send_debug_info(str);			
		}
		
		usleep(500000);
	}
	pthread_exit(NULL);
}


int create_wireless_debug_thread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0, Wireless_Debug_Thread_Func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}

