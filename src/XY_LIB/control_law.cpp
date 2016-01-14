#include "control_law.h"
#include "range.h"

#define DT 						(0.02)
#define HOVER_POINT_RANGE 		(0.1)
#define HOVER_VELOCITY_MIN 		(0.1)
#define TRANS_TO_HOVER_DIS 		(13.0) 
#define DIS_DIFF_WITH_MARK		(0.20)
#define MIN_VEL_TO_GET_IMAGE	(0.3)
#define MIN_ANGLE_TO_GET_IMAGE  (0.1)




#define UPDOWN_CTRL_KP			(0.2)
#define HEIGHT_CTRL_DELTA		(0.5)
#define P2P_MAX_ATTITUDE		(10.0)
#define MAX_CTRL_VEL_UPDOWN_WITH_IMAGE	(0.5)
#define PI						(3.1415926)

extern struct debug_info debug_package;
extern pthread_mutex_t debug_mutex;
extern pthread_cond_t debug_cond;


void init_g_origin_pos(api_pos_data_t *_g_origin_pos)
{
	_g_origin_pos->longti = ORIGIN_IN_HENGSHENG_LONGTI;
	_g_origin_pos->lati = ORIGIN_IN_HENGSHENG_LATI;
	_g_origin_pos->alti = ORIGIN_IN_HENGSHENG_ALTI;
}


/* ´óµØ->ÇòÐÄ */
void geo2XYZ(api_pos_data_t pos, XYZ *pXYZ)
{
	double a = 6378137;				//aÎªÍÖÇòµÄ³¤°ëÖá:a=6378.137km
	double b = 6356752.3141;			//bÎªÍÖÇòµÄ¶Ì°ëÖá:a=6356.7523141km
	double H = pos.alti;	//delete"+a"by zhanglei 0108
	double e = sqrt(1-pow(b ,2)/pow(a ,2));//eÎªÍÖÇòµÄµÚÒ»Æ«ÐÄÂÊ  
	double B = pos.lati;
	double L = pos.longti;
	double W = sqrt(1-pow(e ,2)*pow(sin(B) ,2));
	double N = a/W; //NÎªÍÖÇòµÄÃ®ÓÏÈ¦ÇúÂÊ°ë¾¶ 
	
	pXYZ->x = (N+H)*cos(B)*cos(L);
	pXYZ->y = (N+H)*cos(B)*sin(L);
	pXYZ->z = (N*(1-pow(e ,2))+H)*sin(B);
}

/*ÇòÐÄ->Õ¾ÐÄ add by zhanglei, 20151224*/
/*´Ë×ª»»Ëã·¨xÖáÏò±±£¬yÖáÏò¶«*/
void XYZ2xyz(api_pos_data_t s_pos, XYZ pXYZ, Center_xyz *pxyz)
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

/*Trans current quaternion to angle*/
void QUA2ANGLE(api_quaternion_data_t cur_quaternion, Body_Angle *body_angle) 
{

	body_angle->roll_deg= 180/PI*atan2(2*(cur_quaternion.q0*cur_quaternion.q1+cur_quaternion.q2*cur_quaternion.q3),1-2*(cur_quaternion.q1*cur_quaternion.q1+cur_quaternion.q2*cur_quaternion.q2));
	body_angle->pitch_deg= 180/PI*asin(2*(cur_quaternion.q0*cur_quaternion.q2-cur_quaternion.q3*cur_quaternion.q1));
	body_angle->yaw_deg= 180/PI*atan2(2*(cur_quaternion.q0*cur_quaternion.q3+cur_quaternion.q1*cur_quaternion.q2),1-2*(cur_quaternion.q2*cur_quaternion.q2+cur_quaternion.q3*cur_quaternion.q3));
}


/*get down to a certain height with IMAGE and have velocity at the end*/
int XY_Cal_Attitude_Ctrl_Data_Down_To_Height_WithVel_IMAGE(	api_vel_data_t cvel, 
														api_pos_data_t cpos,
														api_pos_data_t _focus_point,
														float target_vel, 
														float t_height,
														attitude_data_t *puser_ctrl_data,
														int *flag)
{
	
	double kp_z = UPDOWN_CTRL_KP;  //Up down cotrol factor, add by zhanglei 1225
	
	//¶¨µã¿ØÖÆ£¬µ«²»×ö¸ß¶È¿ØÖÆ
	XY_Cal_Vel_Ctrl_Data_Get_Down_With_IMAGE(cvel, cpos, puser_ctrl_data); //´Ë´¦epos.heightÉèÖÃµÄ¸ß¶ÈÖµÃ»ÓÐÒâÒå£¬²»×ö¸ß¶È¿ØÖÆ

	puser_ctrl_data->thr_z = kp_z * (t_height - cpos.height);   //¿ØÖÆµÄÊÇ´¹Ö±ËÙ¶È

	if(fabs(puser_ctrl_data->thr_z) < fabs(target_vel))  //±£³ÖÄ©¶ËµÄËÙ¶È£¬add 0111 by zhanglei
	{
		puser_ctrl_data->thr_z = target_vel;
//		printf("%.f, %.f",puser_ctrl_data->thr_z, target_vel);
	}

#if 1
	//addÏÞ·ù´úÂë
	if(puser_ctrl_data->thr_z > 1.0)		//modified from 2 to 1 by zhanglei 0111
	{
		puser_ctrl_data->thr_z = 1.0;
	}
	else if(puser_ctrl_data->thr_z < (-1.0) )
	{
		puser_ctrl_data->thr_z = -1.0;
	}
	
#endif	
	//addÊ¹ÓÃ³¬Éù²¨´úÂë
	

	if(fabs(t_height - cpos.height) < HEIGHT_CTRL_DELTA)//modified the para, add fabs, by zhanglei 1225
	{
		*flag = 1;
	}

	//XY_Debug_Sprintf(0, "\n[Height] %.8f.\n", cpos.height);
	
}


/*up to certain height with velocity at the end*/
int XY_Cal_Attitude_Ctrl_Data_Up_To_Height_WithVel(	api_vel_data_t cvel, 
														api_pos_data_t cpos,
														api_pos_data_t _focus_point,
														float target_vel, 
														float t_height,
														attitude_data_t *puser_ctrl_data,
														int *flag)
{
	
	double kp_z = UPDOWN_CTRL_KP;  //Up down cotrol factor, add by zhanglei 1225
	
	//in up mode, use the Fix point control without IMAGE info, FP not control the height, the height ctrl is done in this func
	XY_Cal_Vel_Ctrl_Data_FP(cvel, cpos, _focus_point, _focus_point, -1, puser_ctrl_data); //not ctrl height

	puser_ctrl_data->thr_z = kp_z * (t_height - cpos.height);   //¿ØÖÆµÄÊÇ´¹Ö±ËÙ¶È

	if(fabs(puser_ctrl_data->thr_z) < fabs(target_vel))  //±£³ÖÄ©¶ËµÄËÙ¶È£¬add 0111 by zhanglei
	{
		puser_ctrl_data->thr_z = target_vel;
//		printf("%.f, %.f",puser_ctrl_data->thr_z, target_vel);
	}

#if 1
	//addÏÞ·ù´úÂë
	if(puser_ctrl_data->thr_z > 1.0)		//modified from 2 to 1 by zhanglei 0111
	{
		puser_ctrl_data->thr_z = 1.0;
	}
	else if(puser_ctrl_data->thr_z < (-1.0) )
	{
		puser_ctrl_data->thr_z = -1.0;
	}
	
#endif	
	//addÊ¹ÓÃ³¬Éù²¨´úÂë
	

	if(fabs(t_height - cpos.height) < HEIGHT_CTRL_DELTA)//modified the para, add fabs, by zhanglei 1225
	{
		*flag = 1;
	}

	//XY_Debug_Sprintf(0, "\n[Height] %.8f.\n", cpos.height);
	
}


/*up down to fix height with no velocity at the end*/
int XY_Cal_Attitude_Ctrl_Data_UpDown_TO_Height(	api_vel_data_t cvel,
												api_pos_data_t cpos,
												api_pos_data_t _focus_point,
												float t_height,
												attitude_data_t *puser_ctrl_data,
												int *flag)
{
	double kp_z = UPDOWN_CTRL_KP;  //Up down cotrol factor, add by zhanglei 1225
	
	//¶¨µã¿ØÖÆ£¬µ«²»×ö¸ß¶È¿ØÖÆ
	XY_Cal_Vel_Ctrl_Data_FP(cvel, cpos, _focus_point, _focus_point, -1, puser_ctrl_data); //´Ë´¦epos.heightÉèÖÃµÄ¸ß¶ÈÖµÃ»ÓÐÒâÒå£¬²»×ö¸ß¶È¿ØÖÆ

	puser_ctrl_data->thr_z = kp_z * (t_height - cpos.height); 

	//addÏÞ·ù´úÂë
	if(puser_ctrl_data->thr_z > 2.0)
	{
		puser_ctrl_data->thr_z = 2.0;
	}
	else if(puser_ctrl_data->thr_z < (-2.0) )
	{
		puser_ctrl_data->thr_z = -2.0;
	}
	//addÊ¹ÓÃ³¬Éù²¨´úÂë


	if(fabs(t_height - cpos.height) < HEIGHT_CTRL_DELTA)//modified the para, add fabs, by zhanglei 1225
	{
		*flag = 1;
	}

	//XY_Debug_Sprintf(0, "\n[Height] %.8f.\n", cpos.height);

}




int XY_Cal_Attitude_Ctrl_Data_P2P(	api_vel_data_t cvel, 
									api_pos_data_t cpos, 
									float height, 
									Leg_Node *p_legn,
									attitude_data_t *puser_ctrl_data,
									int *flag)
{
	double k1d, k1p, k2d, k2p;
	static api_pos_data_t epos, spos;	
	static XYZ eXYZ, cXYZ;
	static Center_xyz exyz, cxyz;
	api_pos_data_t g_origin_pos;
	static int count = 0;
	double last_distance_xyz = 0.0;
	volatile double thetaC, phiC;
	volatile double thetaC1, thetaC2, phiC1, phiC2;
	char msg[100];
	static int i = 0;

	//set the origin with Longti&Lati
	init_g_origin_pos(&g_origin_pos);
	
/*	//¸ú×Ùµã¿ØÖÆ²ÎÊý, last modified by zhanglei, 1222
	k1d=0.5;
	k1p=1;		//1222 by zhanglei µ÷ÕûÔö¼Ó10±¶£¬·ÉÐÐ²âÊÔok£¬Ö®Ç°Îª·ÂÕæokµÄ²ÎÊý0.1
	k2d=0.5;
	k2p=1;		//1222 by zhanglei µ÷ÕûÔö¼Ó10±¶£¬·ÉÐÐ²âÊÔok£¬Ö®Ç°Îª·ÂÕæokµÄ²ÎÊý0.1
*/
	
	if(count == 0 || count == 2)
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

	//´Ó´óµØ×ø±êÏµ×ª»»µ½ÇòÐÄ×ø±êÏµ
	geo2XYZ(epos, &eXYZ);
	geo2XYZ(cpos, &cXYZ);
	//use origin as the center to transfer, the same as start
	//XYZ2xyz(g_origin_pos, eXYZ, &exyz);
	//XYZ2xyz(g_origin_pos, cXYZ, &cxyz);
	XYZ2xyz(spos, eXYZ, &exyz);
	XYZ2xyz(spos, cXYZ, &cxyz);

	exyz.x -= DELTA_X_M_GOOGLEEARTH;
	exyz.y -= DELTA_Y_M_GOOGLEEARTH;
	exyz.z -= DELTA_Z_M_GOOGLEEARTH;

	//Õ¾ÐÄ×ø±êÏµµÄ¿ØÖÆ²ÎÊýadd by zhanglei, 1225;
	k1d=0.5;
	k1p=1;		
	k2d=0.5;
	k2p=1;	

	//Õ¾ÐÄ×ø±êÏµÏÂ½øÐÐ×ËÌ¬¿ØÖÆadd by zhanglei, 1225;
	thetaC= k1p*(cxyz.x-exyz.x)+k1d*cvel.x;
	phiC=-k2p*(cxyz.y-exyz.y)-k2d*cvel.y;

	//ÏÞ·ù
	if(thetaC>P2P_MAX_ATTITUDE)
	{
		thetaC=P2P_MAX_ATTITUDE;
	}
	else if(thetaC<(-1)*P2P_MAX_ATTITUDE)
	{
		thetaC=(-1)*P2P_MAX_ATTITUDE;
	}

	if(phiC>P2P_MAX_ATTITUDE)
	{
		phiC=P2P_MAX_ATTITUDE;
	}
	else if(phiC<(-1)*P2P_MAX_ATTITUDE)
	{
		phiC=(-1)*P2P_MAX_ATTITUDE;
	}


/*	
	//ÔÚÇòÐÄ×ø±êÏµÏÂ¸ú×Ùµã×ËÌ¬¿ØÖÆ, add by zhanglei, 1224; simulated 1225am ok by zl.
    thetaC = k1p*(cXYZ.z-eXYZ.z) + k1d*cvel.x;//ÆÚÍûµÄ¸©Ñö½Ç XYZÇòÐÄ×ø±êÏµ£¬ZÖáNorth+,+thetaC_pitch²úÉúSouthËÙµÝ
	phiC = k2p*(cXYZ.x-eXYZ.x) - k2d*cvel.y;//ÆÚÍûµÄ¹ö×ª½Ç XYZÇòÐÄ×ø±êÏµ£¬XÖáWest+, +phiC_roll²úÉúEastËÙ¶È
*/

	puser_ctrl_data->ctrl_flag=0x00;//´¹Ö±ËÙ¶È£¬Ë®Æ½×ËÌ¬£¬º½Ïò½Ç¶È¿ØÖÆÄ£Ê½
	puser_ctrl_data->roll_or_x = phiC;			//¹ö×ª½Ç.»úÌåxÖá¡£°´ÕÕÄ¿Ç° Ground×ø±êÏµ£¬²úÉúyÖáËÙ¶È
	puser_ctrl_data->pitch_or_y = thetaC;		//¸©Ñö½Ç.»úÌåyÖá¡£°´ÕÕÄ¿Ç° Ground×ø±êÏµ£¬²úÉú-xÖáËÙ¶È
	puser_ctrl_data->thr_z =  height - cpos.height;   // ¸ß¶Èµ¥Î»¸º·´À¡¿ØÖÆ£¬ºóÆÚ¿Éµ÷Õû·´À¡ÏµÊýÓÅ»¯ÐÔÄÜ -z 
	puser_ctrl_data->yaw = 0;

//	last_distance=sqrt(pow((-1)*(cXYZ.x- eXYZ.x), 2)+pow((cXYZ.z-eXYZ.z), 2));//XÖáÔÚ¶«°ëÇòÏòÎ÷ÎªÕý£¬ÔÚxÖáÔö¼Ó¸ººÅ

	last_distance_xyz=sqrt(pow((cxyz.x- exyz.x), 2)+pow((cxyz.y-exyz.y), 2));

#if 0
		//printf("Dis--> X:%.8lf, Y:%.8lf\n",(cxyz.x- exyz.x), (cxyz.y-exyz.y));//xÖáÔÚ¶«°ëÇòÏòÎ÷ÎªÕý£¬ÔÚxÖáÔö¼Ó¸ººÅ
		printf("Dis: %lf\n", last_distance_xyz);
		printf("End Long, Lat--> %.9lf.\t%.9lf.\t\n",epos.longti,epos.lati);
		
#endif
	
	if(last_distance_xyz < TRANS_TO_HOVER_DIS)
	{
		*flag = XY_Cal_Vel_Ctrl_Data_FP(cvel, cpos, spos, epos, height, puser_ctrl_data);
		if(*flag == 1)
		{
			count = 2;
		}
		//count = 0;
	}

#if 0
	FP_With_IMAGE();	//test transfer, DELETE after test
#endif

	//XY_Debug_Sprintf(0, "\n[Height] %.8f.\n", cpos.height);
	//XY_Debug_Sprintf(1, "[XYZ] %.8lf, %.8lf.\n", (cxyz.x- exyz.x), (cxyz.y-exyz.y));
	
}


/*=====¶¨µãÐüÍ£¿ØÖÆ============*/
/*Author: zhanglei
/*Create:2015-12-23
/*Last Modify: 2015-12-24 by zhanglei
/*Input: µ±Ç°ËÙ¶È£¬µ±Ç°¾­Î³¶È
/*		Õ¾ÐÄ¾­Î³¶È£¬Ä¿±ê¾­Î³¶È
/*		·µ»Ø¿ØÖÆÁ¿Ö¸Õë
/*Output: ×´Ì¬
=================================*/
int XY_Cal_Vel_Ctrl_Data_FP(api_vel_data_t cvel, api_pos_data_t cpos, api_pos_data_t spos, api_pos_data_t tpos, float height, attitude_data_t *puser_ctrl_data)
{
	double k1d, k1p, k2d, k2p;
	double y_e_vel, x_n_vel;
	double last_distance_XYZ = 0.0;
	double last_distance_xyz = 0.0;
	double last_velocity=0.0;
	static XYZ tXYZ, txyz, cXYZ, cxyz, sXYZ,sxyz;

	//´Ó´óµØ×ø±êÏµ×ª»»µ½ÇòÐÄ×ø±êÏµ
	geo2XYZ(tpos, &tXYZ);
	geo2XYZ(cpos,&cXYZ);

	//´ÓÇòÐÄ×ø±êÏµ×ª»»µ½Õ¾ÐÄ×ø±êÏµ
	XYZ2xyz(spos, cXYZ, &cxyz);
	XYZ2xyz(spos, tXYZ, &txyz);

	//txyz.x -= DELTA_X_M_GOOGLEEARTH;
	//txyz.y -= DELTA_Y_M_GOOGLEEARTH;
	//txyz.z -= DELTA_Z_M_GOOGLEEARTH;

	//1225ÏÂÎç·ÂÕæ½á¹û²ÎÊý£¬by zhanglei ok, 1227·ÉÐÐ²ÎÊýok
	k1d=0.05;
	k1p=0.4;	
	k2d=0.05;
	k2p=0.4;	

	x_n_vel = -k1p*(cxyz.x-txyz.x)-k1d*(cvel.x);//xyzÕ¾ÐÄ×ø±êÏµ£¬xÖá±±ÏòÕý
	y_e_vel = -k2p*(cxyz.y-txyz.y)-k2d*(cvel.y);//xyzÕ¾ÐÄ×ø±êÏµ£¬yÖá¶«ÏòÕý

	puser_ctrl_data->ctrl_flag=0x40;//´¹Ö±ËÙ¶È£¬Ë®Æ½ËÙ¶È£¬º½Ïò½Ç¶È¿ØÖÆÄ£Ê½
	puser_ctrl_data->roll_or_x = x_n_vel;			//x±±ÏòÆÚÍûËÙ¶È
	puser_ctrl_data->pitch_or_y = y_e_vel;		//y¶«ÏòÆÚÍûËÙ¶È

	if(height >= 0)
		puser_ctrl_data->thr_z =  height - cpos.height;   // ¸ß¶Èµ¥Î»¸º·´À¡¿ØÖÆ£¬ºóÆÚ¿Éµ÷Õû·´À¡ÏµÊýÓÅ»¯ÐÔÄÜ 

	last_distance_xyz=sqrt(pow((cxyz.x- txyz.x), 2)+pow((cxyz.y-txyz.y), 2));

#if 0
	//printf("HoverDis--> X:%.8lf, Y:%.8lf\n",(cxyz.x- txyz.x),(cxyz.y-txyz.y));
	printf("last_dis: %lf\n", last_distance_xyz);
	printf("FP  End Long, Lat--> .\t%.9lf.\t%.9lf.\t\n",tpos.longti,tpos.lati);

#endif

	if(last_distance_xyz < HOVER_POINT_RANGE)
	{
//		printf("Ctrl--> X:%.10lf, Y:%.10lf\n",puser_ctrl_data->roll_or_x,puser_ctrl_data->pitch_or_y);
//		puser_ctrl_data->roll_or_x = 0;			//x±±ÏòÆÚÍûËÙ¶È
//		puser_ctrl_data->pitch_or_y = 0;		//y¶«ÏòÆÚÍûËÙ¶È		

		last_velocity=sqrt(cvel.x*cvel.x+cvel.y*cvel.y);
		//printf("last vel%lf\n", last_velocity);

		if(fabs(last_velocity) < HOVER_VELOCITY_MIN)
		{
			//XY_Cal_Vel_Ctrl_Data_Image(cpos.height);	//offset+ËÄÔªÊý+¸ß¶È(cpos.height)
			return 1;
		}
	}
	return 0;
	
}




//Hover in point with IMAGE
void XY_Cal_Vel_Ctrl_Data_FP_With_IMAGE(api_vel_data_t cvel, api_pos_data_t cpos, float height, attitude_data_t *puser_ctrl_data,int *flag)
{
	double k1d, k1p, k2d, k2p;
	api_quaternion_data_t cur_quaternion;
	float yaw_angle;
    float roll_angle,roll_rard;
    float pitch_angle,pitch_rard;
	static Offset offset,offset_adjust;
	float x_camera_diff_with_roll;
	float y_camera_diff_with_pitch;
	double y_e_vel, x_n_vel;
	static float last_dis_to_mark,last_velocity;
	XYZ	cXYZ;
	static Center_xyz cur_target_xyz;
	Center_xyz cxyz;
	static api_pos_data_t target_origin_pos;
	static int target_update=0;
	static int count=0;
	
	
	DJI_Pro_Get_Quaternion(&cur_quaternion);

	roll_rard = atan2(2*(cur_quaternion.q0*cur_quaternion.q1+cur_quaternion.q2*cur_quaternion.q3),1-2*(cur_quaternion.q1*cur_quaternion.q1+cur_quaternion.q2*cur_quaternion.q2));
	pitch_rard= asin(2*(cur_quaternion.q0*cur_quaternion.q2-cur_quaternion.q3*cur_quaternion.q1));
	yaw_angle = 180/PI*atan2(2*(cur_quaternion.q0*cur_quaternion.q3+cur_quaternion.q1*cur_quaternion.q2),1-2*(cur_quaternion.q2*cur_quaternion.q2+cur_quaternion.q3*cur_quaternion.q3));

	roll_angle = 180/PI*roll_rard;
	pitch_angle = 180/PI*pitch_rard;

	//the first time give the origin point
	if(count == 0 || count == 2)
	{
		target_origin_pos=cpos;
		cur_target_xyz.x=0;
		cur_target_xyz.y=0;
		count++;
	}
	

	//steady enough, ready to get image,set new target
	if (sqrt(cvel.x*cvel.x+cvel.y*cvel.y) < MIN_VEL_TO_GET_IMAGE && roll_angle < MIN_ANGLE_TO_GET_IMAGE && pitch_angle < MIN_ANGLE_TO_GET_IMAGE || target_update == 1)
	{		

		if( XY_Get_Offset_Data(&offset, OFFSET_GET_ID_A) == 0)
		{
			
			// modified to "Meter", raw data from image process is "cm"
			offset.x = offset.x/100;
			offset.y = offset.y/100;
			offset.z = offset.z/100;


			//modified the camera offset with attitude angle, --------NOT INCLUDING the YAW, NEED added!!!
			x_camera_diff_with_roll =cpos.height * tan(roll_rard);// modified to use the Height not use offset.z by zl, 0113
			y_camera_diff_with_pitch = cpos.height * tan(pitch_rard);// modified to use the Height not use offset.z by zl, 0113
		
			offset_adjust.x = offset.x - x_camera_diff_with_roll;
			offset_adjust.y = offset.y - y_camera_diff_with_pitch;

	
			//check if close enough to the image target
			if(sqrt(pow(offset_adjust.y, 2)+pow(offset_adjust.x, 2)) < DIS_DIFF_WITH_MARK)
			{
				last_velocity=sqrt(cvel.x*cvel.x+cvel.y*cvel.y);
				
				if(last_velocity < HOVER_VELOCITY_MIN)
				{
					cur_target_xyz.x=0;
					cur_target_xyz.y=0;
					cur_target_xyz.z=0;
					
					*flag=1;
					count = 2;
					return;
				}
					
			}

		//trans to get the xyz coordination
		target_origin_pos=cpos;
		
		//set the target with the image target with xyz
		cur_target_xyz.x =  (-1)*(offset_adjust.y); //add north offset
		cur_target_xyz.y =  offset_adjust.x; //add east offset	
		
		target_update=1;
		
		
		//printf("target x,y-> %.8lf.\t%.8lf.\t%.8lf.\t\n",cur_target_xyz.x,cur_target_xyz.y,last_dis_to_mark);
	}
	/*
	else		//if not steady enough, then control the vel to zero, set target to zero
	{
		cur_target_xyz.x=0;
		cur_target_xyz.y=0;
		cur_target_xyz.z=0;
	}
	*/

		//hover para the same as the FP
		k1d=0.05;
		k1p=0.05;	//0.1 simulation test ok 0113 //set to 0.2 flight test bad, returm to 0.1
		k2d=0.05;
		k2p=0.05;		

		//use the origin updated last time "target_origin_pos", to get the current cxyz
		geo2XYZ(cpos,&cXYZ);
		XYZ2xyz(target_origin_pos, cXYZ, &cxyz);		

		//use the xyz coordination to get the target, the same as the FP control		
		x_n_vel = -k1p*(cxyz.x-cur_target_xyz.x)-k1d*(cvel.x);
		y_e_vel = -k2p*(cxyz.y-cur_target_xyz.y)-k2d*(cvel.y);
		
		//x_n_vel = -k1p*(-1)*(offset_adjust.y)-k1d*(cvel.x);  	//camera y is to the south when DJI head focus north
		//y_e_vel = -k2p*(offset_adjust.x)-k2d*(cvel.y);	//camera x is to the east when DJI head focus north

		puser_ctrl_data->ctrl_flag=0x40;//´¹Ö±ËÙ¶È£¬Ë®Æ½ËÙ¶È£¬º½Ïò½Ç¶È¿ØÖÆÄ£Ê½
		puser_ctrl_data->roll_or_x = x_n_vel;			//x±±ÏòÆÚÍûËÙ¶È
		puser_ctrl_data->pitch_or_y = y_e_vel;		//y¶«ÏòÆÚÍûËÙ¶È

		if(height >= 0)
			puser_ctrl_data->thr_z =  height - cpos.height;   // ¸ß¶Èµ¥Î»¸º·´À¡¿ØÖÆ£¬ºóÆÚ¿Éµ÷Õû·´À¡ÏµÊýÓÅ»¯ÐÔÄÜ 

		last_dis_to_mark=sqrt(pow((cxyz.x- cur_target_xyz.x), 2)+pow((cxyz.y-cur_target_xyz.y), 2));
		//last_dis_to_mark=sqrt(pow(offset_adjust.y, 2)+pow(offset_adjust.x, 2));
		
		if (last_dis_to_mark < HOVER_POINT_RANGE)
			target_update=0;
		
	}
	else
	{
		puser_ctrl_data->roll_or_x = 0;
		puser_ctrl_data->pitch_or_y = 0;
		if(height >= 0)
			puser_ctrl_data->thr_z =  height - cpos.height;
	}
/*
		if (last_dis_to_mark < DIS_DIFF_WITH_MARK)
		{

			last_velocity=sqrt(cvel.x*cvel.x+cvel.y*cvel.y);
			
			if(last_velocity < HOVER_VELOCITY_MIN)
			{
				//update_cur_legn_data(cpos.longti, cpos.lati);
				*flag=1;
			}
			
		}
*/

	

}

/* define in route.cpp */
extern int drone_goback;

void XY_Cal_Vel_Ctrl_Data_Get_Down_With_IMAGE(api_vel_data_t cvel, api_pos_data_t cpos, attitude_data_t *puser_ctrl_data)
{

	double k1d, k1p, k2d, k2p;
	api_quaternion_data_t cur_quaternion;
	float yaw_angle;
    float roll_angle,roll_rard;
    float pitch_angle,pitch_rard;
	static Offset offset,offset_adjust;
	float x_camera_diff_with_roll;
	float y_camera_diff_with_pitch;
	double y_e_vel, x_n_vel;
	static float last_dis_to_mark,last_velocity;
	XYZ	cXYZ;
	static Center_xyz cur_target_xyz;
	Center_xyz cxyz;
	static api_pos_data_t target_origin_pos;
	static int target_update=0;
	static int count=0;
	
	
	DJI_Pro_Get_Quaternion(&cur_quaternion);

	roll_rard = atan2(2*(cur_quaternion.q0*cur_quaternion.q1+cur_quaternion.q2*cur_quaternion.q3),1-2*(cur_quaternion.q1*cur_quaternion.q1+cur_quaternion.q2*cur_quaternion.q2));
	pitch_rard= asin(2*(cur_quaternion.q0*cur_quaternion.q2-cur_quaternion.q3*cur_quaternion.q1));
	yaw_angle = 180/PI*atan2(2*(cur_quaternion.q0*cur_quaternion.q3+cur_quaternion.q1*cur_quaternion.q2),1-2*(cur_quaternion.q2*cur_quaternion.q2+cur_quaternion.q3*cur_quaternion.q3));

	roll_angle = 180/PI*roll_rard;
	pitch_angle = 180/PI*pitch_rard;

	//the first time give the origin point
	if(count==0 || drone_goback == 1)
	{
		target_origin_pos=cpos;
		cur_target_xyz.x=0;
		cur_target_xyz.y=0;
		drone_goback++;
		count++;
	}

	//steady enough, ready to get image,set new target
	if (sqrt(cvel.x*cvel.x+cvel.y*cvel.y) < MIN_VEL_TO_GET_IMAGE && roll_angle < MIN_ANGLE_TO_GET_IMAGE && pitch_angle < MIN_ANGLE_TO_GET_IMAGE || target_update == 1)
	{		

		if( XY_Get_Offset_Data(&offset, OFFSET_GET_ID_A) == 0)
		{
			
			// modified to "Meter", raw data from image process is "cm"
			offset.x = offset.x/100;
			offset.y = offset.y/100;
			offset.z = offset.z/100;


			//modified the camera offset with attitude angle, --------NOT INCLUDING the YAW, NEED added!!!
			x_camera_diff_with_roll =cpos.height * tan(roll_rard);// modified to use the Height not use offset.z by zl, 0113
			y_camera_diff_with_pitch = cpos.height * tan(pitch_rard);// modified to use the Height not use offset.z by zl, 0113
		
			offset_adjust.x = offset.x - x_camera_diff_with_roll;
			offset_adjust.y = offset.y - y_camera_diff_with_pitch;

	
			//check if close enough to the image target
			if(sqrt(pow(offset_adjust.y, 2)+pow(offset_adjust.x, 2)) < DIS_DIFF_WITH_MARK)
			{
				last_velocity=sqrt(cvel.x*cvel.x+cvel.y*cvel.y);
				
				if(last_velocity < HOVER_VELOCITY_MIN)
				{
					cur_target_xyz.x=0;
					cur_target_xyz.y=0;
					cur_target_xyz.z=0;
				}
					
			}

		//trans to get the xyz coordination
		target_origin_pos=cpos;
		
		//set the target with the image target with xyz
		cur_target_xyz.x =  (-1)*(offset_adjust.y); //add north offset
		cur_target_xyz.y =  offset_adjust.x; //add east offset	
		
		target_update=1;
		
		
		printf("target x,y-> %.8lf.\t%.8lf.\t%.8lf.\t\n",cur_target_xyz.x,cur_target_xyz.y,last_dis_to_mark);
		}
	/*
	else		//if not steady enough, then control the vel to zero, set target to zero
	{
		cur_target_xyz.x=0;
		cur_target_xyz.y=0;
		cur_target_xyz.z=0;
	}
	*/

		//hover para the same as the FP
		k1d=0.05;
		k1p=0.05;	//simulation test 0113;adjust to 0.05,flight test ok 0114
		k2d=0.05;
		k2p=0.05;		

		//use the origin updated last time "target_origin_pos", to get the current cxyz
		geo2XYZ(cpos,&cXYZ);
		XYZ2xyz(target_origin_pos, cXYZ, &cxyz);		

		//use the xyz coordination to get the target, the same as the FP control		
		x_n_vel = -k1p*(cxyz.x-cur_target_xyz.x)-k1d*(cvel.x);
		y_e_vel = -k2p*(cxyz.y-cur_target_xyz.y)-k2d*(cvel.y);

		//lower the x y control
		if(x_n_vel > MAX_CTRL_VEL_UPDOWN_WITH_IMAGE)
		{
			x_n_vel=MAX_CTRL_VEL_UPDOWN_WITH_IMAGE;
		}else if (x_n_vel < (-1) * MAX_CTRL_VEL_UPDOWN_WITH_IMAGE)
		{
			x_n_vel= (-1) * MAX_CTRL_VEL_UPDOWN_WITH_IMAGE;
		}

		if(y_e_vel > MAX_CTRL_VEL_UPDOWN_WITH_IMAGE)
		{
			y_e_vel=MAX_CTRL_VEL_UPDOWN_WITH_IMAGE;
		}else if (y_e_vel < (-1) * MAX_CTRL_VEL_UPDOWN_WITH_IMAGE)
		{
			y_e_vel= (-1) * MAX_CTRL_VEL_UPDOWN_WITH_IMAGE;
		}

		
		//printf("Ctrl_X, Y-->.\t.\t%f.\t%f\n", x_n_vel,y_e_vel);

		
		//x_n_vel = -k1p*(-1)*(offset_adjust.y)-k1d*(cvel.x);  	//camera y is to the south when DJI head focus north
		//y_e_vel = -k2p*(offset_adjust.x)-k2d*(cvel.y);	//camera x is to the east when DJI head focus north

		puser_ctrl_data->ctrl_flag=0x40;//´¹Ö±ËÙ¶È£¬Ë®Æ½ËÙ¶È£¬º½Ïò½Ç¶È¿ØÖÆÄ£Ê½
		puser_ctrl_data->roll_or_x = x_n_vel;			//x±±ÏòÆÚÍûËÙ¶È
		puser_ctrl_data->pitch_or_y = y_e_vel;		//y¶«ÏòÆÚÍûËÙ¶È

		last_dis_to_mark=sqrt(pow((cxyz.x- cur_target_xyz.x), 2)+pow((cxyz.y-cur_target_xyz.y), 2));
		//last_dis_to_mark=sqrt(pow(offset_adjust.y, 2)+pow(offset_adjust.x, 2));
		
		if (last_dis_to_mark < HOVER_POINT_RANGE)
			target_update=0;
		
	}
	else
	{
		puser_ctrl_data->roll_or_x = 0;
		puser_ctrl_data->pitch_or_y = 0;
	}

}



/*
	»ñÈ¡³¬Éù²¨Êý¾ÝµÄ·½Ê½

	float data;
	if(XY_Get_Ultra_Data(&data) == 0)
   	{
		»ñÈ¡µÄÊý¾ÝÓÐÐ§
		¿ÉÒÔ¶Ôdata½øÐÐ·ÖÎö
   	}

	»ñÈ¡×ËÌ¬ËÄÔªÊý
	api_quaternion_data_t cur_quaternion;
	DJI_Pro_Get_Quaternion(&cur_quaternion);

	»ñÈ¡Í¼Ïñoffset
	Offset offset;
	if( XY_Get_Offset_Data(&offset) == 0)	//·µ»Ø0±íÊ¾Êý¾ÝÓÐÐ§
			printf("Get Offset - x:%.4f, y:%.4f, z:%.3f\n", offset.x, offset.y, offset.z);

	´ò¿ª/¹Ø±ÕÉãÏñÍ·
	XY_Start_Capture();
	XY_Stop_Capture();

 */

