#include "control_law.h"

#define DT 0.02
#define HOVER_POINT_RANGE 0.1

extern struct debug_info debug_package;
extern pthread_mutex_t debug_mutex;
extern pthread_cond_t debug_cond;


/* 大地->球心 */
void geo2XYZ(api_pos_data_t pos, XYZ *pXYZ)
{
	double a = 6378137;				//a为椭球的长半轴:a=6378.137km
	double b = 6356752.3141;			//b为椭球的短半轴:a=6356.7523141km
	double H = pos.alti + a;
	double e = sqrt(1-pow(b ,2)/pow(a ,2));//e为椭球的第一偏心率  
	double B = pos.lati;
	double L = pos.longti;
	double W = sqrt(1-pow(e ,2)*pow(sin(B) ,2));
	double N = a/W; //N为椭球的卯酉圈曲率半径 
	
	pXYZ->x = (N+H)*cos(B)*cos(L);
	pXYZ->y = (N+H)*cos(B)*sin(L);
	pXYZ->z = (N*(1-pow(e ,2))+H)*sin(B);
}

/*球心->站心 add by zhanglei, 20151224*/
/*此转换算法x轴向北，y轴向东*/
void XYZ2xyz(api_pos_data_t s_pos, XYZ pXYZ, Center_xyz*pxyz)
{  	
	XYZ s_XYZ, temp_XYZ;

	geo2XYZ(s_pos, &s_XYZ);

	temp_XYZ.x=pXYZ.x-s_XYZ.x;
	temp_XYZ.y=pXYZ.y-s_XYZ.y;
	temp_XYZ.z=pXYZ.z-s_XYZ.z;

	pxyz->x = -sin(s_pos.lati)*cos(s_pos.longti)*temp_XYZ.x - sin(s_pos.lati)*sin(s_pos.longti)*temp_XYZ.y + cos(s_pos.lati)*temp_XYZ.z;
	pxyz->y = -sin(s_pos.longti)*temp_XYZ.x + cos(s_pos.longti)*temp_XYZ.y; 
	pxyz->z = cos(s_pos.lati)*cos(s_pos.longti)*temp_XYZ.x + cos(s_pos.lati)*sin(s_pos.longti)*temp_XYZ.y + sin(s_pos.lati)*temp_XYZ.z;  //delete "-a"

}


int XY_Cal_Attitude_Ctrl_Data_UpDown(api_vel_data_t cvel, api_pos_data_t cpos, float t_height, attitude_data_t *puser_ctrl_data, int *flag)
{
	puser_ctrl_data->roll_or_x = 0;
	puser_ctrl_data->pitch_or_y = 0; 	 
	double kp_z;


	if(t_height == LOW_HEIGHT)	//is down
	{
		
	}

	//Up down cotrol factor, add by zhanglei 1225
	kp_z=0.2;

	puser_ctrl_data->thr_z = kp_z * (t_height - cpos.height);   //控制的是垂直速度

	if(fabs(t_height - cpos.height) < 0.5)//modified the para, add fabs, by zhanglei 1225
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
	static XYZ eXYZ, cXYZ;
	static Center_xyz exyz, cxyz;
	static int count = 0;
	double last_distance_xyz = 0.0;
	volatile double thetaC, phiC;
	volatile double thetaC1, thetaC2, phiC1, phiC2;
	char msg[100];
	static int i = 0;


/*	//跟踪点控制参数, last modified by zhanglei, 1222
	k1d=0.5;
	k1p=1;		//1222 by zhanglei 调整增加10倍，飞行测试ok，之前为仿真ok的参数0.1
	k2d=0.5;
	k2p=1;		//1222 by zhanglei 调整增加10倍，飞行测试ok，之前为仿真ok的参数0.1
*/
	
	if(count == 0)
	{
		epos.longti = p_legn->leg.end._longti;
		epos.lati = p_legn->leg.end._lati;
		epos.alti = p_legn->leg.current._alti;

		spos.longti=cpos.longti;
		spos.lati=cpos.lati;
		spos.alti=cpos.alti;
		spos.height=cpos.height;
		count++;
	}

	//从大地坐标系转换到球心坐标系
	geo2XYZ(epos, &eXYZ);
	geo2XYZ(cpos,&cXYZ);
	//从球心坐标系转到站心坐标系add by zhanglei, 1225;
	XYZ2xyz(spos,eXYZ,&exyz);
	XYZ2xyz(spos,cXYZ,&cxyz);

	//站心坐标系的控制参数add by zhanglei, 1225;
	k1d=0.5;
	k1p=1;		
	k2d=0.5;
	k2p=1;	

	//站心坐标系下进行姿态控制add by zhanglei, 1225;
	thetaC= k1p*(cxyz.x-exyz.x)+k1d*cvel.x;
	phiC=-k2p*(cxyz.y-exyz.y)-k2d*cvel.y;

/*	
	//在球心坐标系下跟踪点姿态控制, add by zhanglei, 1224; simulated 1225am ok by zl.
    thetaC = k1p*(cXYZ.z-eXYZ.z) + k1d*cvel.x;//期望的俯仰角 XYZ球心坐标系，Z轴North+,+thetaC_pitch产生South速递
	phiC = k2p*(cXYZ.x-eXYZ.x) - k2d*cvel.y;//期望的滚转角 XYZ球心坐标系，X轴West+, +phiC_roll产生East速度
*/
#if 0
	i++;
	if(i>10)
	{
		i = 0;
		thetaC1 = k1p*(cXYZ.z-eXYZ.z);	
		thetaC2 = k1d*cvel.x;
		phiC1 = k2p*(cXYZ.x-eXYZ.x)
		phiC2 = - k2d*cvel.y;
	
		sprintf(msg, "\n[%.8lf] [%.8lf] [%.8lf] [%.8lf]\n", thetaC1, thetaC2, phiC1, phiC2);
		XY_Debug_Easy_Send(msg, strlen((const char *)msg));
	}
#endif

	puser_ctrl_data->ctrl_flag=0x00;//垂直速度，水平姿态，航向角度控制模式
	puser_ctrl_data->roll_or_x = phiC;			//滚转角.机体x轴。按照目前 Ground坐标系，产生y轴速度
	puser_ctrl_data->pitch_or_y = thetaC;		//俯仰角.机体y轴。按照目前 Ground坐标系，产生-x轴速度
	puser_ctrl_data->thr_z =  height - cpos.height;   // 高度单位负反馈控制，后期可调整反馈系数优化性能 -z 
	puser_ctrl_data->yaw = 0;

//	last_distance=sqrt(pow((-1)*(cXYZ.x- eXYZ.x), 2)+pow((cXYZ.z-eXYZ.z), 2));//X轴在东半球向西为正，在x轴增加负号

	last_distance_xyz=sqrt(pow((cxyz.x- exyz.x), 2)+pow((cxyz.y-exyz.y), 2));


	XY_Debug_Get_UserCtrl(&debug_package.user_ctrl_data, 	puser_ctrl_data->roll_or_x,
															puser_ctrl_data->pitch_or_y,
															puser_ctrl_data->thr_z,
															puser_ctrl_data->yaw);
	XY_Debug_Get_Pos(&debug_package.cur, cxyz.x,  cxyz.y,  cxyz.z);
	XY_Debug_Get_Last_Dist(&debug_package.dist, last_distance_xyz);

	pthread_mutex_lock(&debug_mutex);
	pthread_cond_signal(&debug_cond);	//释放条件变量
	pthread_mutex_unlock(&debug_mutex);

#if 0
		printf("Dis--> X:%.8lf, Y:%.8lf\n",(cxyz.x- exyz.x), (cxyz.y-exyz.y));//x轴在东半球向西为正，在x轴增加负号
#endif
	
		if(last_distance_xyz < 5)
		{
			*flag = XY_Cal_Vel_Ctrl_Data_FP(cvel, cpos, spos, epos, height, puser_ctrl_data);
			count = 0;
		}
	
}


/*=====定点悬停控制============*/
/*Author: zhanglei
/*Create:2015-12-23
/*Last Modify: 2015-12-24 by zhanglei
/*Input: 当前速度，当前经纬度
/*		站心经纬度，目标经纬度
/*		返回控制量指针
/*Output: 状态
=================================*/
int XY_Cal_Vel_Ctrl_Data_FP(api_vel_data_t cvel, api_pos_data_t cpos, api_pos_data_t spos, api_pos_data_t tpos, float height, attitude_data_t *puser_ctrl_data)
{
	double k1d, k1p, k2d, k2p;
	double y_e_vel, x_n_vel;
	double last_distance_XYZ = 0.0;
	double last_distance_xyz = 0.0;
	static XYZ tXYZ, txyz, cXYZ, cxyz, sXYZ,sxyz;

#if 0
	//定点控制参数, last modified by zhanglei, 1224, simulated ok 
	k1d=0.4;
	k1p=1;	
	k2d=0.4;
	k2p=1;
#endif

	//从大地坐标系转换到球心坐标系
	geo2XYZ(tpos, &tXYZ);
	geo2XYZ(cpos,&cXYZ);
	
#if 0
	//在球心坐标系下定点速度控制, add by zhanglei, 1224, simulated ok

	x_n_vel=-k1p*(cXYZ.z-tXYZ.z)-k1d*(cvel.x);//XYZ球心坐标系，Z轴北向正
	y_e_vel=k2p*(cXYZ.x-tXYZ.x)-k2d*(cvel.y);//XYZ球心坐标系，X轴在东半球向西为正，第一项由负改为正


	puser_ctrl_data->ctrl_flag=0x40;//垂直速度，水平速度，航向角度控制模式
	puser_ctrl_data->roll_or_x = x_n_vel;			//x北向期望速度
	puser_ctrl_data->pitch_or_y = y_e_vel;		//y东向期望速度
	puser_ctrl_data->thr_z =  height - cpos.height;   // 高度单位负反馈控制，后期可调整反馈系数优化性能 

	last_distance_XYZ=sqrt(pow((-1)*(cXYZ.x- tXYZ.x), 2)+pow((cXYZ.z-tXYZ.z), 2));//x轴在东半球向西为正，在x轴增加负号

	if(last_distance_XYZ < HOVER_POINT_RANGE)
		return 1;
	else
		return 0;


#endif

#if 1
	//从球心坐标系转换到站心坐标系
	XYZ2xyz(spos, cXYZ, &cxyz);
	XYZ2xyz(spos, tXYZ, &txyz);

	//1225下午仿真结果参数，by zhanglei ok
	k1d=0.05;
	k1p=0.4;	
	k2d=0.05;
	k2p=0.4;


/*	//1225上午最后重复试验的参数
	k1d=0.05;
	k1p=0.5;	
	k2d=0.05;
	k2p=0.5;
*/
	x_n_vel=-k1p*(cxyz.x-txyz.x)-k1d*(cvel.x);//xyz站心坐标系，x轴北向正
	y_e_vel=-k2p*(cxyz.y-txyz.y)-k2d*(cvel.y);//xyz站心坐标系，y轴东向正

	puser_ctrl_data->ctrl_flag=0x40;//垂直速度，水平速度，航向角度控制模式
	puser_ctrl_data->roll_or_x = x_n_vel;			//x北向期望速度
	puser_ctrl_data->pitch_or_y = y_e_vel;		//y东向期望速度
	puser_ctrl_data->thr_z =  height - cpos.height;   // 高度单位负反馈控制，后期可调整反馈系数优化性能 

	last_distance_xyz=sqrt(pow((cxyz.x- txyz.x), 2)+pow((cxyz.y-txyz.y), 2));

#if 0
	printf("HoverDis--> X:%.8lf, Y:%.8lf\n",(cxyz.x- txyz.x),(cxyz.y-txyz.y));
#endif

	if(last_distance_xyz < HOVER_POINT_RANGE)
	{
		
		//XY_Cal_Vel_Ctrl_Data_Image(cpos.height);	//offset+四元数+高度(cpos.height)
		return 1;
	}
	else
		return 0;
#endif
	
}


int XY_Cal_Vel_Ctrl_Data_Image(float height)
{
	
	Offset offset;
	api_quaternion_data_t cur_quaternion;
	
	
	/* get offset */
	if(read_refresh_flag() == 1)
	{
		clear_refresh_flag();
		offset = get_stored_offset_block();
		printf("%f %f %f\n", offset.x, offset.y, offset.z);
	}
	else 
	{
	
	}
	/* get quaternion */
	DJI_Pro_Get_Quaternion(&cur_quaternion);
	
	
	
}



