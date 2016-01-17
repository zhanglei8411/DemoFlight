#ifndef __SD_STORE_H__
#define __SD_STORE_H__


#include <time.h>
#include <sys/time.h> 
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <math.h>
#include "../DJI_LIB/DJI_Pro_App.h"


int XY_SD_Log_Setup(void);
void set_attitude_data(attitude_data_t _user_ctrl_data);
static void *store_to_log_thread_func(void * arg);
int XY_Start_Store_Log(void);
int XY_Stop_Store_Log(void);


#endif
