#ifndef __XY_ROUTE_H__
#define __XY_ROUTE_H__

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>
#include "../DJI_LIB/DJI_Pro_App.h"
#include "XY_LL.h"
#include "XY_Ctrl.h"

#define _WL_MAX_RECV_SIZE          (1024)
#define SAFETY_HEIGHT				(10.0)	//m
#define LOW_HEIGHT					(0.5)	//m

typedef struct
{
	unsigned short recv_index;
	unsigned char comm_recv_buf[_WL_MAX_RECV_SIZE];
}Wl_Filter;


#pragma  pack(1)

typedef struct
{
	unsigned char sof;
	unsigned int length;
	unsigned char leg_num;
}Wl_Header;


#pragma  pack()



void byte_stream_handler(Wl_Filter* p_filter, unsigned char in_data);

int Xy_Waiting_Task_Arrived(void);
int Xy_Setup_List_Head(void);
int Xy_Start_Express_Task_Thread(pthread_t *_thread);
FILE* Xy_Open_Data_File(void);



#endif
