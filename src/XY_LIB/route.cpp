#include "route.h"
#include "thread_common_op.h"
#include "image_identify.h"
#include "sd_store_log.h"
#include "wireless_debug.h"
#include "control_law.h"

Leg_Node *route_head = NULL;
Leg_Node *cur_legn = NULL;
Leg task_info;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
extern struct debug_info debug_package;

int drone_goback = 0;

api_pos_data_t focus_point;
static void *XY_Aircraft_UpDown_Thread_Func(void * arg)
{
	api_vel_data_t cur_vel;
	api_pos_data_t cur_pos;
	
	attitude_data_t user_ctrl_data;
	int flag = 0;

	/* focus, focus, focus */
	user_ctrl_data.ctrl_flag = 0x40;//modified from 00 to 40 by zhanglei 1225
	
	user_ctrl_data.roll_or_x = 0;//控制水平速度为0, add by zhanglei 1225
	user_ctrl_data.pitch_or_y = 0;	

	// add by zhouzibo in 16/1/11
	switch((int)(*(int *)arg))
	{
		//Up
		case MODE_1_UP_TO_U2_IMAGE_VEL_1:				
			DJI_Pro_Get_Pos(&focus_point);
			break;
		case MODE_2_UP_TO_U3:
			DJI_Pro_Get_Pos(&focus_point);
			break;

		//Down
		case MODE_3_DOWN_TO_D3:
			DJI_Pro_Get_Pos(&focus_point);
			break;

			case MODE_T1_TEST_HOVER_WITH_IMAGE:
			DJI_Pro_Get_Pos(&focus_point);
			break;	

			case MODE_4_DOWN_TO_D2_IMAGE_VEL_N05:
			DJI_Pro_Get_Pos(&focus_point);
			break;	

		
		case MODE_5_DOWN_TO_D1:
			DJI_Pro_Get_Pos(&focus_point);
			break;	
		default:
			break;
	}
	
	while(1)
	{
		/* 1. get cur height from M100 */
		DJI_Pro_Get_Pos(&cur_pos);
		/* 2. get cur vel from M100 */
		DJI_Pro_Get_GroundVo(&cur_vel);

		if( (int)(*(int *)arg) == MODE_1_UP_TO_U2_IMAGE_VEL_1 )
		{
			/* up with image vel=1 */
			XY_Cal_Attitude_Ctrl_Data_Up_To_Height_WithVel(	cur_vel,
																cur_pos,
																focus_point,
																UP_TO_HEIGHT_WITH_VEL_AT_END, GO_UP_TO_HEIGHT_WTIH_IMAGE_H_U2, &user_ctrl_data, &flag);
		}
		else if( (int)(*(int *)arg) == MODE_2_UP_TO_U3)
		{
			/* up to U3  */
			XY_Cal_Attitude_Ctrl_Data_UpDown_TO_Height(	cur_vel,
														cur_pos,
														focus_point,
														GO_UP_TO_CRUICE_HEIGHT_H_U3, &user_ctrl_data, &flag);
		}
		else if( (int)(*(int *)arg) == MODE_3_DOWN_TO_D3)
		{
			/* down to D3*/
			XY_Cal_Attitude_Ctrl_Data_UpDown_TO_Height(	cur_vel,
														cur_pos,
														focus_point,
														GO_DOWN_TO_HEIGHT_H_D3, &user_ctrl_data, &flag);
		}

		//temp test the image hover fun
		else if( (int)(*(int *)arg) == MODE_T1_TEST_HOVER_WITH_IMAGE)
		{
			/* Hover to find mark */
			XY_Cal_Vel_Ctrl_Data_FP_With_IMAGE(	cur_vel, cur_pos, focus_point.height, &user_ctrl_data, &flag);
		}


		else if( (int)(*(int *)arg) == MODE_4_DOWN_TO_D2_IMAGE_VEL_N05)
		{
			/* down with image vel=-0.5 */
			XY_Cal_Attitude_Ctrl_Data_Down_To_Height_WithVel_IMAGE(	cur_vel,
																cur_pos,
																focus_point,
																DOWN_TO_HEIGHT_WITH_VEL_AT_END, GO_DOWN_TO_HEIGHT_WITH_IMAGE_H_D2, &user_ctrl_data, &flag);
		}
		else if( (int)(*(int *)arg) == MODE_5_DOWN_TO_D1 )
		{
			/* down to D1, ready to landing  */
			 XY_Cal_Attitude_Ctrl_Data_UpDown_TO_Height(cur_vel,
			 											cur_pos,
			 											focus_point, 
			 											GO_DOWN_TO_HEIGHT_READY_TO_LAND_H_D1, &user_ctrl_data, &flag);
		}		


		if(flag == 1)
		{
			goto exit;
		}
		/* 4. use attitude ctrl data to change the action of drone */
		DJI_Pro_Attitude_Control(&user_ctrl_data);
		
		set_attitude_data(user_ctrl_data);
		
		/* 5. sleep 20ms */
		usleep(20000);
		
	}

exit:
	pthread_mutex_lock(&mutex);
	pthread_cond_signal(&cond);	//释放条件变量
	pthread_mutex_unlock(&mutex);
	
	pthread_exit(NULL);

}

static void *XY_Aircraft_P2P_Thread_Func(void * arg)
{
	api_vel_data_t cur_vel;
	api_pos_data_t cur_pos;
	attitude_data_t user_ctrl_data;
	int flag = 0;

	/* focus, focus, focus */
	user_ctrl_data.ctrl_flag = 0x00;

	
	while(1)
	{
		/* 1. get cur height from M100 */
		DJI_Pro_Get_Pos(&cur_pos);
		/* 2. get cur vel from M100 */
		DJI_Pro_Get_GroundVo(&cur_vel);
		/* 3. set cur pos in cur_legn */
		set_leg_cur_pos(&cur_legn->leg, cur_pos.longti, cur_pos.lati, cur_pos.alti);
		
		/* 4. cal attitude data by cur vel, cur pos and goal pos  */
		//Cal_Attitude_Ctrl_Data(cur_vel, cur_pos, cur_legn, &user_ctrl_data, &flag);
		/* 
		 * 4.1 根据cur_pos中的当前经纬度高度和cur_legn中的目标点经纬度高度，计算三维空间上的距离
		 * 4.2 剩余距离是否小于某个允许值
		 * 是:flag置一,退出
		 * 否:根据剩余距离和当前速度计算user_ctrl_data
		 */
		XY_Cal_Attitude_Ctrl_Data_P2P(cur_vel, cur_pos, GO_UP_TO_CRUICE_HEIGHT_H_U3, cur_legn, &user_ctrl_data, &flag);

		if(flag == 1)
		{
			goto exit;
		}
		/* 5. use attitude ctrl data to change the action of drone */
		DJI_Pro_Attitude_Control(&user_ctrl_data);
		
		set_attitude_data(user_ctrl_data);
		
		/* 6. sleep 20ms */
		usleep(20000);
		
	}

exit:
	pthread_mutex_lock(&mutex);
	pthread_cond_signal(&cond);
	pthread_mutex_unlock(&mutex);
	
	pthread_exit(NULL);

}


static void *Route_Task_Thread_Func(void * arg)
{
	int ret;
	pthread_t A_ARR;
	int Updown_Mode = MODE_1_UP_TO_U2_IMAGE_VEL_1;//first up stage add by zhanglei 0110

	temporary_init_route_list();
	
	if(route_head->next == NULL)
	{
		goto error;
	}
	cur_legn = route_head->next;

	while(1)
	{
		/* --------------------- START ------------------------- */
		/* 起飞完成后高度 - 1.27m */
		/* 1.1 aircraft take off */
		ret = DJI_Pro_Status_Ctrl(4,0);
		/* 1.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);	//一旦进入wait, 会释放mutex的lock, 等收到signal信号就会自动重新获取mutex的lock	
		pthread_mutex_unlock(&mutex);
		/* 1.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */		


//Need Add image data
		XY_Start_Capture();
		/* --------------------- START ------------------------- */
		/* 2.1 aircraft up with image to U2*/
		Updown_Mode = MODE_1_UP_TO_U2_IMAGE_VEL_1;
		if( pthread_create(&A_ARR, 0, XY_Aircraft_UpDown_Thread_Func, &Updown_Mode) != 0  )
		{
			goto error;
		}
		/* tip in runlog and send to wireless terminal */
		printf("----------------- Up to 10 meters -----------------\n");
		XY_Debug_Sprintf(1, "Up to 10 meters\n");
		
		/* 2.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 2.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */
		XY_Stop_Capture();

		
		/* --------------------- START ------------------------- */
		/* 3.1 aircraft up to U3 ready to cruise*/
		Updown_Mode = MODE_2_UP_TO_U3;
		if( pthread_create(&A_ARR, 0, XY_Aircraft_UpDown_Thread_Func, &Updown_Mode) != 0  )
		{
			goto error;
		}
		/* tip in runlog and send to wireless terminal */
		printf("----------------- Up to 40 meters -----------------\n");
		XY_Debug_Sprintf(1, "Up to 40 meters\n");
		
		/* 3.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 3.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */


		
		
		/* --------------------- START ------------------------- */
		/* 4.1 point to piont leg fly */				
		if( pthread_create(&A_ARR, 0, XY_Aircraft_P2P_Thread_Func, NULL) != 0  )
		{
			goto error;
		}
		/* tip in runlog and send to wireless terminal */
		printf("----------------- p2p start -----------------\n");
		XY_Debug_Sprintf(1, "p2p start\n");
		/* 4.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 4.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */
		



		
		/* --------------------- START ------------------------- */
		/* 5.1 aircraft down to GO_DOWN_TO_HEIGHT_H_D3 */
		Updown_Mode = MODE_3_DOWN_TO_D3;
		if( pthread_create(&A_ARR, 0, XY_Aircraft_UpDown_Thread_Func, &Updown_Mode) != 0  )
		{
			goto error;
		}
		/* tip in runlog and send to wireless terminal */
		printf("----------------- Down to 15 meters -----------------\n");
		XY_Debug_Sprintf(1, "Down to 15 meters\n");
		/* 5.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 5.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */


//Need Add image data

		/* --------------------- START ------------------------- */
		XY_Start_Capture();
		/* 6. At D3 height to find mark */
		Updown_Mode = MODE_T1_TEST_HOVER_WITH_IMAGE;
		if( pthread_create(&A_ARR, 0, XY_Aircraft_UpDown_Thread_Func, &Updown_Mode) != 0  )
		{
			goto error;
		}
		/* tip in runlog and send to wireless terminal */
		printf("----------------- Start to identifies image -----------------\n");
		XY_Debug_Sprintf(1, "Start to identifies image\n");
		/* 6.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 6.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */


		
		/* --------------------- START ------------------------- */
		/* 7.1 aircraft down to GO_DOWN_TO_HEIGHT_WITH_IMAGE_H_D2 with image */
		Updown_Mode = MODE_4_DOWN_TO_D2_IMAGE_VEL_N05;
		if( pthread_create(&A_ARR, 0, XY_Aircraft_UpDown_Thread_Func, &Updown_Mode) != 0  )
		{
			goto error;
		}
		/* tip in runlog and send to wireless terminal */
		printf("----------------- Down to 1 meters -----------------\n");
		XY_Debug_Sprintf(1, "Down to 1 meters\n");
		/* 7.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 7.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */
		XY_Stop_Capture();
		

		/* --------------------- START ------------------------- */
		/* 8.1 aircraft down to D1 */
		Updown_Mode = MODE_5_DOWN_TO_D1;
		if( pthread_create(&A_ARR, 0, XY_Aircraft_UpDown_Thread_Func, &Updown_Mode) != 0  )
		{
			goto error;
		}
		/* tip in runlog and send to wireless terminal */
		printf( "----------------- Down to 0.25 meters -----------------\n");
		XY_Debug_Sprintf(1, "Down to 0.25 meters\n");
		/* 8.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 5.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */


		
		/* --------------------- START ------------------------- */
		
		/* 9. aircraft at D2 ready to landing*/
		ret = DJI_Pro_Status_Ctrl(6,0);
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);

		/* ---------------------- END -------------------------- */





		if(cur_legn->next != NULL)
		{
			cur_legn = cur_legn->next;
		}
		else
		{
			drone_goback = 1;
			goto exit;
		}
	}

error:
	//回收资源
	XY_Release_List_Resource();
exit:
	pthread_exit(NULL);
}


int XY_Start_Route_Task_Thread(pthread_t *_thread)
{

	if(XY_Create_Thread(Route_Task_Thread_Func, NULL, THREADS_ROUTE, -1, SCHED_RR, 99) < 0)
	{
		printf("Create Route Task Thread Error.\n");
		return -1;
	}

	*_thread = get_tid(THREADS_ROUTE);
	return 0;
#if 0
	int ret;
	
	ret = pthread_create(_thread, 0, Route_Task_Thread_Func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
#endif
}

void XY_Release_List_Resource(void)
{
	if(route_head)
	{
		delete_leg_list(route_head);
		printf("Memory of list is free.\n");
	}
}



int temporary_init_route_list(void)
{
	int ret = 0;
	static api_pos_data_t start_pos;
	static int go_back = 0;
	api_pos_data_t g_origin_pos;
	XYZ g_origin_XYZ, start_XYZ;
	Center_xyz g_origin_xyz, start_xyz,g_origin_xyz_app;
	
	route_head = create_head_node(); 
	if(route_head == NULL)
	{
		printf("Create Route Head ERROR.\n");
		return -1;
	}
	
	set_leg_seq(&task_info, 1);
	set_leg_num(&task_info, 1);


	if(go_back == 0)
	{
		start_pos.longti = 0;
		do{
			DJI_Pro_Get_Pos(&start_pos);
			XY_Debug_Send_At_Once("Getting start pos\n");
		}while(start_pos.longti == 0);

		set_leg_start_pos(&task_info, start_pos.longti, start_pos.lati, 0.100000);

		//将终点坐标转为地面坐标系,zhanglei add 0111, not finish


		//原点在羽毛球场起飞点
		g_origin_pos.longti = ORIGIN_IN_HENGSHENG_LONGTI;
		g_origin_pos.lati = ORIGIN_IN_HENGSHENG_LATI;
		g_origin_pos.alti = ORIGIN_IN_HENGSHENG_ALTI;
		
		//从大地坐标系转换到球心坐标系
		geo2XYZ(g_origin_pos, &g_origin_XYZ);
		geo2XYZ(start_pos, &start_XYZ);

		//从球心坐标系转到站心坐标系
		XYZ2xyz(g_origin_pos, start_XYZ, &g_origin_xyz);
		XYZ2xyz(g_origin_pos, start_XYZ, &start_xyz);
	

		/*地面坐标系x轴向北，y轴向东*/
		g_origin_xyz_app.x=g_origin_xyz.x-DELTA_X_M_GOOGLEEARTH;  
		g_origin_xyz_app.y=g_origin_xyz.y-DELTA_Y_M_GOOGLEEARTH;
		g_origin_xyz_app.z=g_origin_xyz.z-DELTA_Z_M_GOOGLEEARTH;		
		
		XY_Debug_Send_At_Once("[g_origin_xyz.x %.3lf,g_origin_xyz.y %.3lf,g_origin_xyz.z %.3lf]\n", g_origin_xyz.x,g_origin_xyz.y,g_origin_xyz.z);
		XY_Debug_Send_At_Once("[start_xyz.x %.3lf,start_xyz.y %.3lf,start_xyz.z %.3lf]\n", start_xyz.x,start_xyz.y,start_xyz.z);
		XY_Debug_Send_At_Once("[g_origin_xyz_app.x %.3lf,g_origin_xyz_app.y %.3lf,g_origin_xyz_app.z %.3lf]\n", g_origin_xyz_app.x,g_origin_xyz_app.y,g_origin_xyz_app.z);
		
		//设置终点位置, not finish
		
		//set_leg_end_pos(&task_info, start_pos.longti - 0.000001, start_pos.lati, 0.100000);// zhanglei 0109 from 1 to 2
		set_leg_end_pos(&task_info, XYI_TERRACE_LONGTI, XYI_TERRACE_LATI, XYI_TERRACE_ALTI);


		go_back = 1;
	}
	else if(go_back == 1)
	{
		set_leg_end_pos(&task_info, start_pos.longti, start_pos.lati, 0.100000);
		start_pos.longti = 0;
		do{
			DJI_Pro_Get_Pos(&start_pos);
			XY_Debug_Send_At_Once("Getting start pos\n");
		}while(start_pos.longti == 0);
		set_leg_start_pos(&task_info, start_pos.longti, start_pos.lati, 0.100000);
		go_back = -1;
	}
	
	
	
	
	printf("Initial information: (%.8lf, %.8lf) to (%.8lf, %.8lf)\n", 	task_info.start._longti, 
																		task_info.start._lati,
																		task_info.end._longti,
																		task_info.end._lati);
	ret = insert_new_leg_into_route_list(route_head, task_info);
	if(ret != 0)
	{
		printf("Add Route Node ERROR.\n");
		return -1;
	}

	return 0;
}


int setup_route_list_head_node(Leg_Node *head)
{
	head = create_head_node();
	if(head == NULL)
		return -1;
	return 0;
}


int insert_new_leg_into_route_list(Leg_Node *head, struct Leg leg)
{
	return add_leg_node(head, leg);
}


Leg_Node *create_head_node(void)
{
	Leg_Node *_head = NULL;

	_head = (Leg_Node *)calloc(1, sizeof(Leg_Node));
	if(_head == NULL)
	{
		return NULL;
	}
	_head->leg.leg_seq = 0;
	_head->next = NULL;

	return _head;
}

int add_leg_node(Leg_Node *_head, struct Leg _leg)
{
	Leg_Node *pcur, *pnew;
	int valid_seq;
	
	/* 1. 定位到尾节点 */
	pcur = _head;
	while(pcur->next)
	{
		pcur = pcur->next;
	}
	/* 2. 判断预插入的节点的数据是否合法 */
	valid_seq = pcur->leg.leg_seq+1;
	if(_leg.leg_seq != valid_seq)
	{
		return -1;
	}
	pnew = (Leg_Node *)calloc(1, sizeof(Leg_Node));
	if(pnew == NULL)
		return -1;
	
	set_leg_seq(&(pnew->leg), _leg.leg_seq);
	set_leg_num(&(pnew->leg), _leg.leg_num);
	set_leg_start_pos(&(pnew->leg), _leg.start._longti, _leg.start._lati, _leg.start._alti);
	set_leg_end_pos(&(pnew->leg), _leg.end._longti, _leg.end._lati, _leg.end._alti);
	
	pcur->next = pnew;
	pnew->next = NULL;

	return 0;
}

void delete_leg_list(Leg_Node *_head)
{
	Leg_Node *p;

	while(_head->next)
	{
		p = _head;
		_head = _head->next;
		free(p);
	}
	free(_head);
	_head = p = NULL;	//防止出现野指针

}

int update_cur_legn_data(double _longti, double _lati)
{
	if(!cur_legn)
		return -1;

	cur_legn->leg.end._longti = _longti;
	cur_legn->leg.end._alti = _lati;
	return 0;
}

