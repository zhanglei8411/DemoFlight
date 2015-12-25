#ifndef __WIRELESS_DEBUG_H__
#define __WIRELESS_DEBUG_H__

#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include "common/uart_common.h"
#include "common/common.h"


#define DEBUG_DEV_NAME "/dev/ttyTHS0"


typedef struct
{
    float 	roll_or_x;
    float	pitch_or_y;
    float	thr_z;
    float	yaw;
}attitude;

struct debug_info{
	position start;
	position end;
	position cur;
	attitude user_ctrl_data;
	double dist;
};

int XY_Debug_Setup(void);
int XY_Debug_Easy_Send(char *buf, int len);
void XY_Debug_Get_Pos(position *pp, double _longti, double _lati, double _alti);
void XY_Debug_Get_UserCtrl(attitude *pa, float _roll, float _pitch, float _thr, float _yaw);
void XY_Debug_Get_Last_Dist(double *dst, double _last_distance);
int sdk_wireless_debug_setup(const char *device, int baudrate);
int create_wireless_debug_thread(void);



#endif
