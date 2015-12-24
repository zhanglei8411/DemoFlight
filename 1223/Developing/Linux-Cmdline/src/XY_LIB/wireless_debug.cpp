#include "wireless_debug.h"

static int debug_fd = -1;
static fd_set debug_fd_set;

struct debug_info debug_package = {0};
pthread_mutex_t debug_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t debug_cond = PTHREAD_COND_INITIALIZER;


int XY_Debug_Setup(void)
{
	int wireless_baudrate = 9600;

	return sdk_wireless_debug_setup(DEBUG_DEV_NAME, wireless_baudrate);
}

int XY_Debug_Easy_Send(char *buf, int len)
{
	return write(debug_fd, buf, len);
}

void XY_Debug_Get_Pos(position *pp, double _longti, double _lati, double _alti)
{
	pp->_longti = _longti;
	pp->_lati = _lati;
	pp->_alti = _alti;
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
			sprintf(str, "\n[S](%.8lf, %.8lf) [Cur](%.8lf, %.8lf)", 	debug_package.start._longti,
																		debug_package.start._lati,  
																		debug_package.cur._longti,
																		debug_package.cur._lati);
			break;
		case 2:
#if 0
			sprintf(str, "[User Ctrl] roll:%.8f, pitch:%.8f, thr:%.8f, yaw:%.8f\n", debug_package.user_ctrl_data.roll_or_x,
																					debug_package.user_ctrl_data.pitch_or_y, 
																					debug_package.user_ctrl_data.thr_z, 
																					debug_package.user_ctrl_data.yaw);
#endif


			break;
		case 3:
#if 0
			sprintf(str, "[Cur](%.8lf, %.8lf, %.8lf) - [Last Distance] %.8lfm\n",	debug_package.cur.longti,
																					debug_package.cur.lati, 
																					debug_package.cur.alti,
																					debug_package.dist);
#endif
			sprintf(str, "\t[Last Distance] %.8lfm\n",	debug_package.dist);
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
		sdk_handle_debug_info(debug_package, str, 3);
		sdk_send_debug_info(str);	
		usleep(1000000);
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

