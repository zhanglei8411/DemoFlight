//============================================================================
// Name        : main.cpp
// Author      : wuyuwei
// Version     :
// Copyright   : DJI Inc
// Description : DJI Onboard API test in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include "DJI_Pro_Sample.h"
#include "XY_LIB/wireless_debug.h"
#include "XY_LIB/route.h"
#include "XY_LIB/common/common.h"
#include "XY_LIB/control_steer.h"
#include "XY_LIB/range.h"
#include "XY_LIB/image_identify.h"

#define DEBUG_PRINTF 1

using namespace std;
extern Aircraft_Status aircraft_status;

void handle_signal(int signum)
{
	switch(signum)
	{
		case SIGINT:
			printf("SIGINT\n");		
			raise(SIGKILL);
			break;
		default:
			break;
	}
}

int main(int argc,char **argv)
{
	int main_operate_code = 0;
	int temp32;
	bool valid_flag = false;
	bool err_flag = false;
	activate_data_t user_act_data; 
	char temp_buf[65];
	char app_bundle_id[32] = "1234567890";
	pthread_t express_thread_id;
	int i = 0;
	int wait_time = 0;


	if(XY_Ctreate_Capture_and_Identify_Thread() < 0)
	{
		printf("Create Capture and identify thread error.\n");
	}
#if 0
	while(1)
	{
		if(read_refresh_flag() == 1)
		{
			clear_refresh_flag();
			offset = get_stored_offset_block();
			printf("main: %f %f %f\n", offset.x, offset.y, offset.z);
		}
	}
#endif
	
	signal(SIGINT, handle_signal);

	if(argc == 2 && strcmp(argv[1],"-v") == 0)
	{
		printf("\nDJI Onboard API Cmdline Test,Ver 1.0.0\n\n");
		return 0;
	}
	printf("\nDJI Onboard API Cmdline Test,Ver 1.1.0\n\n");

	if(DJI_Sample_Setup() < 0)
	{
		printf("Serial Port Open ERROR\n");
		return 0;
	}

	user_act_data.app_key = temp_buf;
	user_act_data.app_ver = SDK_VERSION;
	strcpy((char*)user_act_data.app_bundle_id, app_bundle_id);
    if(DJI_Pro_Get_Cfg(NULL,NULL,&user_act_data.app_id,&user_act_data.app_api_level,user_act_data.app_key) == 0)
	{
		/* user setting */
		printf("--------------------------\n");
		printf("app id=%d\n",user_act_data.app_id);
		printf("app api level=%d\n",user_act_data.app_api_level);
		printf("app key=%s\n",user_act_data.app_key);
		printf("--------------------------\n");
	}
	else
	{
		printf("ERROR:There is no user account\n");	
		return 0;
	}

	XY_Debug_Setup();
	
	if(argc == 2)
	{
		wait_time = atoi(argv[1]);
	}
	wait_time = wait_time<10 ? 10 : wait_time;
	wait_time = wait_time>90 ? 90 : wait_time;
		
	printf("wait_time is %d\n", wait_time);

	KS103_Setup("/dev/ttyTHS2", 9600);	//UART3

	char debug_msg[100];
#if 1		
	while(i < wait_time)
	{
		sleep(1);
		i++;
		if(i % 2 != 0)
			continue;
		sprintf(debug_msg, "[Count down] %d\n", (wait_time - i));
		XY_Debug_Easy_Send(debug_msg, strlen((const char *)debug_msg));
	}
#endif

	if(0 == XY_Load_Goods())
	{
		
		sprintf(debug_msg, "Load succeed\n");
	}
	else
	{
		sprintf(debug_msg, "Load failed\n");
	}
	XY_Debug_Easy_Send(debug_msg, strlen((const char *)debug_msg));
	
	sleep(5);
	
	init_aircraft_status();

	while(1)
	{
		DJI_Pro_Activate_API(&user_act_data,NULL);
		while(aircraft_status.activation == -1);
		if(aircraft_status.activation == 1)
			break;
		sprintf(debug_msg, "Activation failed, return %d\n", aircraft_status.activation);
		XY_Debug_Easy_Send(debug_msg, strlen((const char *)debug_msg));
		aircraft_status.activation = -1;
		usleep(50000);
	}

	sprintf(debug_msg, "Activation succeed, start to request controller.\n");
	XY_Debug_Easy_Send(debug_msg, strlen((const char *)debug_msg));
	
	while(1)
	{
		DJI_Pro_Control_Management(1,NULL);
		while(aircraft_status.obtained_control == -1);
		if(aircraft_status.obtained_control == 1)
			break;
		sprintf(debug_msg, "Obtain control, return %d\n", aircraft_status.obtained_control);
		XY_Debug_Easy_Send(debug_msg, strlen((const char *)debug_msg));
		aircraft_status.obtained_control = -1;
		usleep(50000);
	}
	
	sprintf(debug_msg, "Obtained controller, start to flight.\n");
	XY_Debug_Easy_Send(debug_msg, strlen((const char *)debug_msg));

	int call_cnt = 0;
	while(call_cnt < 2)
	{
		XY_Start_Route_Task_Thread(&express_thread_id);
		pthread_join(express_thread_id, NULL);
		call_cnt++;
		if(call_cnt == 1)
		{
			if(0 == XY_Unload_Goods())
			{
				sprintf(debug_msg, "Unload succeed\n");
			}
			else
			{
				sprintf(debug_msg, "Unload failed\n");
			}	
			XY_Debug_Easy_Send(debug_msg, strlen((const char *)debug_msg));	
			sleep(5);
		}
	}
	
	sprintf(debug_msg, "Task is over controller, pthread already to exit.\n");
	XY_Debug_Easy_Send(debug_msg, strlen((const char *)debug_msg));



	while(1);
	return 0;
}
