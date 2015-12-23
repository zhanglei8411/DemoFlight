#include "control_law.h"

#define DT 0.02

extern struct debug_info debug_package;
extern pthread_mutex_t debug_mutex;
extern pthread_cond_t debug_cond;

volatile double pgx,pgy;
volatile double phiC,thetaC;


int XY_Cal_Attitude_Ctrl_Data_UpDown(api_vel_data_t cvel, api_pos_data_t cpos, float height, attitude_data_t *puser_ctrl_data, int *flag)
{
	puser_ctrl_data->roll_or_x = 0;
	puser_ctrl_data->pitch_or_y = 0; 	 

	puser_ctrl_data->thr_z =  (height - cpos.height);   //控制的是垂直速度

	if(puser_ctrl_data->thr_z < 0.5)
	{
		*flag = 1;
	}
	
	XY_Debug_Get_UserCtrl(&debug_package.user_ctrl_data, 	puser_ctrl_data->roll_or_x,
															puser_ctrl_data->pitch_or_y,
															puser_ctrl_data->thr_z,
															puser_ctrl_data->yaw);
	XY_Debug_Get_Pos(&debug_package.cur, cpos.longti, cpos.lati, cpos.alti);
	XY_Debug_Get_Last_Dist(&debug_package.dist, 0);
	
	pthread_mutex_lock(&debug_mutex);
	pthread_cond_signal(&debug_cond);	//释放条件变量
	pthread_mutex_unlock(&debug_mutex);
}

int XY_Cal_Attitude_Ctrl_Data_P2P(api_vel_data_t cvel, api_pos_data_t cpos, float height, Leg_Node *p_legn, attitude_data_t *puser_ctrl_data, int *flag)
{
	double k1d, k1p, k2d, k2p;
	static api_pos_data_t epos, spos;
	static int count = 0;
	double last_distance = 0.0;
	volatile double thetaC1, thetaC2, phiC1, phiC2;
	char msg[100];
	static int i = 0;

	k1d=0.5;
	k1p=0.1;
	k2d=0.5;
	k2p=0.1;

	
	if(count == 0)
	{
		epos.longti = p_legn->leg.end._longti;
		epos.lati = p_legn->leg.end._lati;
		epos.alti = p_legn->leg.current._alti;
		count++;
	}
	
    thetaC = k1p*(cpos.lati- epos.lati)*1000000 + k1d*cvel.x;			//期望的俯仰角 
	phiC = -k2p*(cpos.longti- epos.longti)*1000000 - k2d*cvel.y;		//期望的滚转角

	i++;
	if(i>10)
	{
		i = 0;
		thetaC1 = k1p*(cpos.lati- epos.lati)*1000000;	
		thetaC2 = k1d*cvel.x;
		phiC1 = -k2p*(cpos.longti- epos.longti)*1000000;
		phiC2 = -k2d*cvel.y;
	
		sprintf(msg, "\n[%.8lf] [%.8lf] [%.8lf] [%.8lf]\n", thetaC1, thetaC2, phiC1, phiC2);
		XY_Debug_Easy_Send(msg, strlen((const char *)msg));
	}
	
	puser_ctrl_data->yaw = 0;
	puser_ctrl_data->roll_or_x = phiC;									//传递滚转角 
	puser_ctrl_data->pitch_or_y = thetaC;								//俯仰角 
	puser_ctrl_data->thr_z =  height - cpos.height;   					// 高度单位负反馈控制，后期可调整反馈系数优化性能 -z 

	last_distance = sqrt(pow((cpos.lati- epos.lati)*6000000, 2)+pow((cpos.longti- epos.longti)*6000000, 2));


	XY_Debug_Get_UserCtrl(&debug_package.user_ctrl_data, 	puser_ctrl_data->roll_or_x,
															puser_ctrl_data->pitch_or_y,
															puser_ctrl_data->thr_z,
															puser_ctrl_data->yaw);
	XY_Debug_Get_Pos(&debug_package.cur, cpos.longti, cpos.lati, cpos.alti);
	XY_Debug_Get_Last_Dist(&debug_package.dist, last_distance);

	pthread_mutex_lock(&debug_mutex);
	pthread_cond_signal(&debug_cond);	//释放条件变量
	pthread_mutex_unlock(&debug_mutex);
	
#if 0
	printf("last_distance is %lf\n", last_distance);
#endif

	if(last_distance < 1)
		*flag = 1;
	
}
