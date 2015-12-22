#ifndef __XY_DEBUG_H__
#define __XY_DEBUG_H__


#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>



#define UART1_DEV_NAME	"/dev/ttyTHS0"
#define DEBUG_DEV_NAME	UART1_DEV_NAME


typedef struct
{
	double longti;
	double lati;
	double alti;
}position;

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

int wireless_send(char *buf,int len);
int XY_Debug_Setup(void);
void XY_Debug_Get_Pos(position *pp, double _longti, double _lati, double _alti);
void XY_Debug_Get_UserCtrl(attitude *pa, float _roll, float _pitch, float _thr, float _yaw);
void XY_Debug_Get_Last_Dist(double *dst, double _last_distance);
int create_wireless_debug_thread(void);



#endif

