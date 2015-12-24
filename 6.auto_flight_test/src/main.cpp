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
#include "DJI_Pro_Sample.h"
#include "XY_APP/XY_LL.h"
#include "XY_APP/XY_Route.h"
#include "XY_APP/XY_Debug.h"

using namespace std;

extern Aircraft_Status aircraft_status;
extern struct debug_info debug_package;
int express_task_running = 0;
Link_Leg_Node *leg_head = NULL;
Leg task_info;


void handle_signal(int signum)
{	
	if(signum == SIGINT)
	{
		printf("SIGINT\n");
		if(leg_head)
		{
			Delete_Leg_List(leg_head);
			printf("Memory of list is free.\n");
		}
		raise(SIGKILL);
	}
}


/*
 * 1.1 Framework of drone
 * no wireless, no ultrasonic, just for simulation
 */
int main(int argc,char **argv)
{
	activate_data_t user_act_data; 
	api_pos_data_t start_pos;
	char temp_buf[65];
	char app_bundle_id[32] = "1234567890";
	int ret;
	pthread_t express_thread_id;
	int i = 0;
	int wait_time = 0;
	char str[100] = "Getting start pos\n";

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


	if(argc == 2)
	{
		wait_time = atoi(argv[1]);

	}
	wait_time = wait_time<10 ? 10 : wait_time;
	wait_time = wait_time>90 ? 90 : wait_time;
		
	printf("wait_time is %d\n", wait_time);


	XY_Debug_Setup();
	
	char num[20];
	while(i < wait_time)
	{
		sleep(1);
		i++;
		if(i % 2 != 0)
			continue;
		sprintf(num, "[Time count] %d\n", i);
		wireless_send(num, strlen((const char *)num));
	}




#if 1

	/* 1. Start 
	 * 1.1 Activate 
	 * 1.2 Get_Control
	 */
	DJI_Pro_Activate_API(&user_act_data,NULL);
	DJI_Pro_Control_Management(1,NULL);



	
	Set_Leg_Seq(&task_info, 1);
	Set_Leg_Num(&task_info, 1);
//	Set_Leg_Start_Pos(&task_info, 1.9888662126323109, 0.3933451676812274, 0.100000);
//	Set_Leg_End_Pos(&task_info, 1.9888662126323110, 0.3933451676812275, 0.100000);


//	Set_Leg_Start_Pos(&task_info, 1.988866, 0.393345, 0.100000);
//	Set_Leg_End_Pos(&task_info, 1.988867, 0.393345, 0.100000);
	
	start_pos.longti = 0;
	do{
		DJI_Pro_Get_Pos(&start_pos);
		wireless_send(str, strlen((const char *)str));
	}while(start_pos.longti == 0);
	Set_Leg_Start_Pos(&task_info, start_pos.longti, start_pos.lati, 0.100000);
	Set_Leg_End_Pos(&task_info, start_pos.longti - 0.000001, start_pos.lati + 0.000001, 0.100000);
	XY_Debug_Get_Pos(&debug_package.start, task_info.longti_s, task_info.lati_s, task_info.alti_s);
	XY_Debug_Get_Pos(&debug_package.end, task_info.longti_e, task_info.lati_e, task_info.alti_e);
	
	
	printf("Initial information: (%.8lf, %.8lf) to (%.8lf, %.8lf)\n", 	task_info.longti_s, 
																		task_info.lati_s,
																		task_info.longti_e,
																		task_info.lati_e);


	
	leg_head = Create_Leg_Head();
	if(leg_head == NULL)
	{
		printf("Create leg head ERROR.\n");
		return 0;
	}
	ret = Add_Leg_Node(leg_head, task_info);
	if(ret != 0)
	{
		printf("Add leg node ERROR.\v");
		return 0;
	}
#if 0
	printf("seq is %d, longti_s is %lf, lati_s is %lf.\n", leg_head->next->leg.leg_seq,
														   leg_head->next->leg.longti_s,
														   leg_head->next->leg.lati_s);
#endif
	

#if 0	
	while(1)
	{

		/* 2. Waiting for the task, Parse data and Setup Leg List */
		do
		{
			/* select监听串口，阻塞至任务到达 */
			ret = Xy_Waiting_Task_Arrived();		
		}while(0);
		
		if(ret == 1)
		{
			Xy_Start_Express_Task_Thread();
		}
		
	}
#endif

	Xy_Start_Express_Task_Thread(&express_thread_id);
	pthread_join(express_thread_id, NULL);

	if(leg_head)
	{
		Delete_Leg_List(leg_head);
		printf("Memory of list is free.\n");
	}

	while(1);
#endif

	return 0;
}
