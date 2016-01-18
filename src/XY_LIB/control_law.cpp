#include "control_law.h"
#include "range.h"


extern struct debug_info debug_package;
extern pthread_mutex_t debug_mutex;
extern pthread_cond_t debug_cond;
/* define in route.cpp */
extern int drone_goback;


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
	XY_Cal_Vel_Ctrl_Data_Get_Down_FP_With_IMAGE(cvel, cpos, puser_ctrl_data); //´Ë´¦epos.heightÉèÖÃµÄ¸ß¶ÈÖµÃ»ÓÐÒâÒå£¬²»×ö¸ß¶È¿ØÖÆ

	puser_ctrl_data->thr_z = kp_z * (t_height - cpos.height);   //¿ØÖÆµÄÊÇ´¹Ö±ËÙ¶È

	if(fabs(puser_ctrl_data->thr_z) < fabs(target_vel))  //±£³ÖÄ©¶ËµÄËÙ¶È£¬add 0111 by zhanglei
	{
		puser_ctrl_data->thr_z = target_vel;
//		printf("%.f, %.f",puser_ctrl_data->thr_z, target_vel);
	}

	//addÏÞ·ù´úÂë
	if(puser_ctrl_data->thr_z > DOWN_LIMIT_VEL_WITH_IMAGE)		//modified from 2 to 1 by zhanglei 0111
	{
		puser_ctrl_data->thr_z = DOWN_LIMIT_VEL_WITH_IMAGE;
	}
	else if(puser_ctrl_data->thr_z < (-1.0)*DOWN_LIMIT_VEL_WITH_IMAGE )
	{
		puser_ctrl_data->thr_z = (-1.0)*DOWN_LIMIT_VEL_WITH_IMAGE;
	}
	
	//addÊ¹ÓÃ³¬Éù²¨´úÂë
	

	if(fabs(t_height - cpos.height) < HEIGHT_CTRL_DELTA_FOR_LOW)//modified the para, add fabs, by zhanglei 1225
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



	puser_ctrl_data->thr_z = kp_z * (t_height - cpos.height); 

	if(fabs(puser_ctrl_data->thr_z) < fabs(target_vel)) 
	{
		puser_ctrl_data->thr_z = target_vel;
	}

	if(t_height > HEIGHT_HIGH_ABOVE)
	{
		
		if(puser_ctrl_data->thr_z > UPDOWN_LIMIT_VEL_HIGH)
		{
			puser_ctrl_data->thr_z = UPDOWN_LIMIT_VEL_HIGH;
		}
		else if(puser_ctrl_data->thr_z < (-1.0)*UPDOWN_LIMIT_VEL_HIGH )
		{
			puser_ctrl_data->thr_z = (-1.0)*UPDOWN_LIMIT_VEL_HIGH;
		}

		//check out
		if(fabs(t_height - cpos.height) < HEIGHT_CTRL_DELTA_FOR_HIGH)//
		{
			*flag = 1;
		}
	}
	else
	{
		if(puser_ctrl_data->thr_z > UPDOWN_LIMIT_VEL_LOW)
		{
			puser_ctrl_data->thr_z = UPDOWN_LIMIT_VEL_LOW;
		}
		else if(puser_ctrl_data->thr_z < (-1.0)*UPDOWN_LIMIT_VEL_LOW )
		{
			puser_ctrl_data->thr_z = (-1.0)*UPDOWN_LIMIT_VEL_LOW;
		}
		
		//check out
		if(fabs(t_height - cpos.height) < HEIGHT_CTRL_DELTA_FOR_LOW)//
		{
			*flag = 1;
		}
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
	


	//for the x,y control, not control the height
	XY_Cal_Vel_Ctrl_Data_FP(cvel, cpos, _focus_point, _focus_point, -1, puser_ctrl_data);



	puser_ctrl_data->thr_z = kp_z * (t_height - cpos.height); 

	
	//use diff limit and check out delta at diff height
	if(t_height > HEIGHT_HIGH_ABOVE)
	{
		
		if(puser_ctrl_data->thr_z > UPDOWN_LIMIT_VEL_HIGH)
		{
			puser_ctrl_data->thr_z = UPDOWN_LIMIT_VEL_HIGH;
		}
		else if(puser_ctrl_data->thr_z < (-1.0)*UPDOWN_LIMIT_VEL_HIGH )
		{
			puser_ctrl_data->thr_z = (-1.0)*UPDOWN_LIMIT_VEL_HIGH;
		}

		//check out
		if(fabs(t_height - cpos.height) < HEIGHT_CTRL_DELTA_FOR_HIGH)//
		{
			*flag = 1;
		}
	}
	else
	{
		if(puser_ctrl_data->thr_z > UPDOWN_LIMIT_VEL_LOW)
		{
			puser_ctrl_data->thr_z = UPDOWN_LIMIT_VEL_LOW;
		}
		else if(puser_ctrl_data->thr_z < (-1.0)*UPDOWN_LIMIT_VEL_LOW )
		{
			puser_ctrl_data->thr_z = (-1.0)*UPDOWN_LIMIT_VEL_LOW;
		}
		
		//check out
		if(fabs(t_height - cpos.height) < HEIGHT_CTRL_DELTA_FOR_LOW)//
		{
			*flag = 1;
		}
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
	static int count = 0;
	double last_distance_xyz = 0.0;
	volatile double thetaC, phiC;
	volatile double thetaC1, thetaC2, phiC1, phiC2;
	char msg[100];
	static int i = 0;
	double x_n_vel, y_e_vel;	//north and east
	
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

	geo2XYZ(epos, &eXYZ);
	geo2XYZ(cpos, &cXYZ);
	XYZ2xyz(spos, eXYZ, &exyz);
	XYZ2xyz(spos, cXYZ, &cxyz);

	if(drone_goback == 0 )
	{
		exyz.x -= DELTA_X_M_GOOGLEEARTH;
		exyz.y -= DELTA_Y_M_GOOGLEEARTH;
		exyz.z -= DELTA_Z_M_GOOGLEEARTH;
	}

#if 0
	k1d=0.5;
	k1p=1;		
	k2d=0.5;
	k2p=1;	

	thetaC= k1p*(cxyz.x-exyz.x)+k1d*cvel.x;
	phiC=-k2p*(cxyz.y-exyz.y)-k2d*cvel.y;
#endif 

	k1d=0.05;
	k1p=0.2;	
	k2d=0.05;
	k2p=0.2;	

	x_n_vel = -k1p*(cxyz.x-exyz.x)-k1d*(cvel.x);
	y_e_vel = -k2p*(cxyz.y-exyz.y)-k2d*(cvel.y);

	double result = 0.0;
	if( ( result = sqrt(pow(x_n_vel, 2)+pow(y_e_vel, 2)) ) > P2P_MAX_VEL_N_E)
	{
		x_n_vel *= (P2P_MAX_VEL_N_E/result);
		y_e_vel *= (P2P_MAX_VEL_N_E/result);
	}

#if 0
	if(x_n_vel > P2P_MAX_VEL_N_E)
	{
		x_n_vel = P2P_MAX_VEL_N_E;
	}
	else if(x_n_vel < (-1.0)*P2P_MAX_VEL_N_E)
	{
		x_n_vel = (-1.0)*P2P_MAX_VEL_N_E;
	}

	if(y_e_vel > P2P_MAX_VEL_N_E)
	{
		y_e_vel = P2P_MAX_VEL_N_E;
	}
	else if(y_e_vel < (-1.0)*P2P_MAX_VEL_N_E)
	{
		y_e_vel = (-1.0)*P2P_MAX_VEL_N_E;
	}
#endif
	

	puser_ctrl_data->ctrl_flag = 0x40;
	puser_ctrl_data->roll_or_x = x_n_vel;			
	puser_ctrl_data->pitch_or_y = y_e_vel;	
	puser_ctrl_data->thr_z =  height - cpos.height;  
	puser_ctrl_data->yaw = 0;

	printf("P2P - xvel: %lf, yvel: %lf\n", x_n_vel, y_e_vel);
	
#if 0
	//limit
	if(thetaC>P2P_MAX_ATTITUDE)
	{
		thetaC=P2P_MAX_ATTITUDE;
	}
	else if(thetaC<(-1.0)*P2P_MAX_ATTITUDE)
	{
		thetaC=(-1.0)*P2P_MAX_ATTITUDE;
	}

	if(phiC>P2P_MAX_ATTITUDE)
	{
		phiC=P2P_MAX_ATTITUDE;
	}
	else if(phiC<(-1.0)*P2P_MAX_ATTITUDE)
	{
		phiC=(-1.0)*P2P_MAX_ATTITUDE;
	}

	puser_ctrl_data->ctrl_flag=0x00;
	puser_ctrl_data->roll_or_x = phiC;			
	puser_ctrl_data->pitch_or_y = thetaC;		
	puser_ctrl_data->thr_z =  height - cpos.height;  
	puser_ctrl_data->yaw = 0;
#endif

	last_distance_xyz=sqrt(pow((cxyz.x- exyz.x), 2)+pow((cxyz.y-exyz.y), 2));

#if 1
		printf("Dis: %lf\n", last_distance_xyz);
		//printf("End Long, Lat--> %.9lf.\t%.9lf.\t\n",epos.longti,epos.lati);
		
#endif
	
	if(last_distance_xyz < TRANS_TO_HOVER_DIS)
	{
		*flag = XY_Cal_Vel_Ctrl_Data_FP_JUST_P2P(cvel, cpos, spos, epos, height, puser_ctrl_data);
		if(*flag == 1)
		{
			count = 2;
		}
		//count = 0;
	}

	//XY_Debug_Sprintf(0, "\n[Height] %.8f.\n", cpos.height);
	//XY_Debug_Sprintf(1, "[XYZ] %.8lf, %.8lf.\n", (cxyz.x- exyz.x), (cxyz.y-exyz.y));
	
}


int XY_Cal_Vel_Ctrl_Data_FP(api_vel_data_t cvel, api_pos_data_t cpos, api_pos_data_t spos, api_pos_data_t tpos, float height, attitude_data_t *puser_ctrl_data)
{
	double k1d, k1p, k2d, k2p;
	double y_e_vel, x_n_vel;
	double last_distance_XYZ = 0.0;
	double last_distance_xyz = 0.0;
	double last_velocity=0.0;
	static XYZ tXYZ, txyz, cXYZ, cxyz, sXYZ,sxyz;


	//trans coordination
	geo2XYZ(tpos, &tXYZ);
	geo2XYZ(cpos,&cXYZ);
	XYZ2xyz(spos, cXYZ, &cxyz);
	XYZ2xyz(spos, tXYZ, &txyz);

	//0114 modify kp from 0.4 to 0.2, to reduce the ctrl attitude, not flight test, up down use ok
	k1d=0.05;
	k1p=0.2;	
	k2d=0.05;
	k2p=0.2;	

	x_n_vel = -k1p*(cxyz.x-txyz.x)-k1d*(cvel.x);
	y_e_vel = -k2p*(cxyz.y-txyz.y)-k2d*(cvel.y);

	puser_ctrl_data->ctrl_flag=0x40;
	puser_ctrl_data->roll_or_x = x_n_vel;			
	puser_ctrl_data->pitch_or_y = y_e_vel;		

	if(height >= 0)
		puser_ctrl_data->thr_z =  height - cpos.height; 

	last_distance_xyz=sqrt(pow((cxyz.x- txyz.x), 2)+pow((cxyz.y-txyz.y), 2));

#if 0
	printf("last_dis: %lf\n", last_distance_xyz);
	printf("FP  End Long, Lat--> .\t%.9lf.\t%.9lf.\t\n",tpos.longti,tpos.lati);

#endif

	if(last_distance_xyz < HOVER_POINT_RANGE)
	{

		last_velocity=sqrt(cvel.x*cvel.x+cvel.y*cvel.y);

		if(fabs(last_velocity) < HOVER_VELOCITY_MIN)
		{
			return 1;
		}
	}
	return 0;
	
}

/*======Focus to Point=============*/
/*Author: zhanglei
/*Create:2015-12-23
/*Last Modify: 2015-12-24 by zhanglei
/*Input: Current vel ,pos
/*		origin,target, target height
/*		control 
/*Output:control para
/*============================*/
int XY_Cal_Vel_Ctrl_Data_FP_JUST_P2P(api_vel_data_t cvel, api_pos_data_t cpos, api_pos_data_t spos, api_pos_data_t tpos, float height, attitude_data_t *puser_ctrl_data)
{
	double k1d, k1p, k2d, k2p;
	double y_e_vel, x_n_vel;
	double last_distance_XYZ = 0.0;
	double last_distance_xyz = 0.0;
	double last_velocity=0.0;
	static XYZ tXYZ, txyz, cXYZ, cxyz, sXYZ,sxyz;


	//trans coordination
	geo2XYZ(tpos, &tXYZ);
	geo2XYZ(cpos,&cXYZ);
	XYZ2xyz(spos, cXYZ, &cxyz);
	XYZ2xyz(spos, tXYZ, &txyz);

	//0114 zhanglei add for the same target
	if(drone_goback == 0)
	{		
		txyz.x -= DELTA_X_M_GOOGLEEARTH;
		txyz.y -= DELTA_Y_M_GOOGLEEARTH;
		txyz.z -= DELTA_Z_M_GOOGLEEARTH;
	}
	printf("goback: %d\n", drone_goback);
	

	//0114 modify kp from 0.4 to 0.2, to reduce the ctrl attitude, not flight test;0116flight test ok, a little slow
	k1d=0.05;
	k1p=0.2;	
	k2d=0.05;
	k2p=0.2;	

	x_n_vel = -k1p*(cxyz.x-txyz.x)-k1d*(cvel.x);
	y_e_vel = -k2p*(cxyz.y-txyz.y)-k2d*(cvel.y);

	puser_ctrl_data->ctrl_flag=0x40;
	puser_ctrl_data->roll_or_x = x_n_vel;			
	puser_ctrl_data->pitch_or_y = y_e_vel;		

	if(height >= 0)
		puser_ctrl_data->thr_z =  height - cpos.height; 

	last_distance_xyz=sqrt(pow((cxyz.x- txyz.x), 2)+pow((cxyz.y-txyz.y), 2));

#if 1
	printf("FP - xvel: %lf, yvel: %lf\n", x_n_vel, y_e_vel);
	printf("last_dis: %lf\n", last_distance_xyz);

	//printf("FP  End Long, Lat--> .\t%.9lf.\t%.9lf.\t\n",tpos.longti,tpos.lati);

#endif

	if(last_distance_xyz < (2*HOVER_POINT_RANGE) )
	{

		last_velocity=sqrt(cvel.x*cvel.x+cvel.y*cvel.y);

		if(fabs(last_velocity) < HOVER_VELOCITY_MIN)
		{
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
	static int goback_cnt = 0;
	static int arrive_flag = 1;
	
	
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
	

	//steady enough, ready to get image,set new target------!!NEED to get angle vel to limit
	if (sqrt(cvel.x*cvel.x+cvel.y*cvel.y) < MIN_VEL_TO_GET_IMAGE && roll_angle < MIN_ANGLE_TO_GET_IMAGE && pitch_angle < MIN_ANGLE_TO_GET_IMAGE || target_update == 1)
	{		

		if( XY_Get_Offset_Data(&offset, OFFSET_GET_ID_A) == 0 && arrive_flag == 1)
		{
			arrive_flag = 0;
			// modified to "Meter", raw data from image process is "cm"
			offset.x = offset.x/100;
			offset.y = offset.y/100;
			offset.z = offset.z/100;

			//adjust with the camera install delta dis
			offset.x -= CAM_INSTALL_DELTA_X;
			offset.y -= CAM_INSTALL_DELTA_Y;
			
			//modified the camera offset with attitude angle, --------NOT INCLUDING the YAW, NEED added!!!
			x_camera_diff_with_roll =cpos.height * tan(roll_rard);// modified to use the Height not use offset.z by zl, 0113
			y_camera_diff_with_pitch = cpos.height * tan(pitch_rard);// modified to use the Height not use offset.z by zl, 0113
		
			offset_adjust.x = offset.x - x_camera_diff_with_roll;
			offset_adjust.y = offset.y - y_camera_diff_with_pitch;
			

#if 0
			if(drone_goback > 0 && goback_cnt == 0)
			{
				goback_cnt = 1;
				
			}
#endif
	
			//check if close enough to the image target
			if(sqrt(pow(offset_adjust.y, 2)+pow(offset_adjust.x, 2)) < 2.5)
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

			//add limit to current target, the step is 3m
			if (sqrt(pow(cur_target_xyz.x, 2)+pow(cur_target_xyz.y, 2)) > MAX_EACH_DIS_IMAGE_GET_CLOSE)
			{
				cur_target_xyz.x = cur_target_xyz.x * MAX_EACH_DIS_IMAGE_GET_CLOSE / sqrt(pow(cur_target_xyz.x, 2)+pow(cur_target_xyz.y, 2));
				cur_target_xyz.y = cur_target_xyz.y * MAX_EACH_DIS_IMAGE_GET_CLOSE / sqrt(pow(cur_target_xyz.x, 2)+pow(cur_target_xyz.y, 2));
			}
			
			target_update=1;
		
		
			printf("target x,y-> %.8lf.\t%.8lf.\t%.8lf.\t\n",cur_target_xyz.x,cur_target_xyz.y,last_dis_to_mark);
		}

		//hover to FP, but lower kp to the FP without image
		k1d=0.05;
		k1p=0.1;	//0.1 simulation test ok 0113 //set to 0.2 flight test bad, returm to 0.1
		k2d=0.05;
		k2p=0.1;		

		//use the origin updated last time "target_origin_pos", to get the current cxyz
		geo2XYZ(cpos,&cXYZ);
		XYZ2xyz(target_origin_pos, cXYZ, &cxyz);		

		//use the xyz coordination to get the target, the same as the FP control		
		x_n_vel = -k1p*(cxyz.x-cur_target_xyz.x)-k1d*(cvel.x);
		y_e_vel = -k2p*(cxyz.y-cur_target_xyz.y)-k2d*(cvel.y);
		
		puser_ctrl_data->ctrl_flag=0x40;
		puser_ctrl_data->roll_or_x = x_n_vel;			
		puser_ctrl_data->pitch_or_y = y_e_vel;		

		if(height >= 0)
			puser_ctrl_data->thr_z =  height - cpos.height;  

		last_dis_to_mark=sqrt(pow((cxyz.x- cur_target_xyz.x), 2)+pow((cxyz.y-cur_target_xyz.y), 2));
		//last_dis_to_mark=sqrt(pow(offset_adjust.y, 2)+pow(offset_adjust.x, 2));
		
		//if (last_dis_to_mark < HOVER_POINT_RANGE)
		if(last_dis_to_mark < (1.5) )
		{
			arrive_flag = 1;
			target_update=0;
		}
		
	}
	else
	{
		puser_ctrl_data->roll_or_x = 0;
		puser_ctrl_data->pitch_or_y = 0;
		if(height >= 0)
			puser_ctrl_data->thr_z =  height - cpos.height;
	}

}



void XY_Cal_Vel_Ctrl_Data_Get_Down_FP_With_IMAGE(api_vel_data_t cvel, api_pos_data_t cpos, attitude_data_t *puser_ctrl_data)
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
	static int no_GPS_mode_on = 0;
	static Center_xyz cxyz_no_gps;
	float gps_ctrl_x,gps_ctrl_y,del_ctrl_x_gps,del_ctrl_y_gps;
	api_common_data_t g_acc;
	static api_vel_data_t cvel_no_gps;
	static int integration_count;
	
	
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
		if( drone_goback == 1 )
		{
			drone_goback++;
		}
		
		count++;
	}

	//steady enough, ready to get image,set new target------!!NEED to get angle vel to limit
	if (sqrt(cvel.x*cvel.x+cvel.y*cvel.y) < MIN_VEL_TO_GET_IMAGE && roll_angle < MIN_ANGLE_TO_GET_IMAGE && pitch_angle < MIN_ANGLE_TO_GET_IMAGE || target_update == 1)
	{		

		if( XY_Get_Offset_Data(&offset, OFFSET_GET_ID_A) == 0)
		{
			
			// modified to "Meter", raw data from image process is "cm"
			offset.x = offset.x/100;
			offset.y = offset.y/100;
			offset.z = offset.z/100;
			
			//adjust with the camera install delta dis
			offset.x -= CAM_INSTALL_DELTA_X;
			offset.y -= CAM_INSTALL_DELTA_Y;

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

			
			//add limit to current target, the step is 3m
			if (sqrt(pow(cur_target_xyz.x, 2)+pow(cur_target_xyz.y, 2)) > MAX_EACH_DIS_IMAGE_GET_CLOSE)
			{
				cur_target_xyz.x = cur_target_xyz.x * MAX_EACH_DIS_IMAGE_GET_CLOSE / sqrt(pow(cur_target_xyz.x, 2)+pow(cur_target_xyz.y, 2));
				cur_target_xyz.y = cur_target_xyz.y * MAX_EACH_DIS_IMAGE_GET_CLOSE / sqrt(pow(cur_target_xyz.x, 2)+pow(cur_target_xyz.y, 2));
			}
			
			target_update = 1;

			/*--ADD NO GPS CONTROL STATE----*/
			//judge the GPS health data
			if (cpos.health_flag < GPS_HEALTH_GOOD && no_GPS_mode_on == 0)
			{
				no_GPS_mode_on = 1;
				integration_count = 1;//limit the time length of using nogps mode, add by zhanglei 0118
				
				cxyz_no_gps.x = 0;
				cxyz_no_gps.y = 0;
				
				cvel_no_gps.x = cvel.x;//get current velocity, modi by zhanglei 0117
				cvel_no_gps.y = cvel.y;
			}

			
			printf("target x,y-> %.8lf.\t%.8lf.\t%.8lf.\t\n",cur_target_xyz.x,cur_target_xyz.y,last_dis_to_mark);
		}


		//when target get GPS is not good, start the interation process
		if (no_GPS_mode_on == 1)
		{
			DJI_Pro_Get_GroundAcc(&g_acc);

			//limit the time length of using nogps mode, add by zhanglei 0118
			if(integration_count >= 100) // 2 secend reset the integration.
			{
				no_GPS_mode_on = 0;
			}

			//Integ x,y by velocity
			cxyz_no_gps.x += cvel_no_gps.x * DT;
			cxyz_no_gps.y += cvel_no_gps.y * DT;

			//Integ velocity by acc
			cvel_no_gps.x += g_acc.x * DT;
			cvel_no_gps.y += g_acc.y * DT;

			integration_count++;
			
			printf("Ground acc: (x)%f, (y)%f\n", g_acc.x, g_acc.y);
			
		}
		
		//hover to FP, but lower kp to the FP without image
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


		//when ready to control, judge the GPS, if not good, use the ingteration data as current position
		if  (cpos.health_flag < GPS_HEALTH_GOOD && no_GPS_mode_on == 1)
		{
			gps_ctrl_x = x_n_vel;
			gps_ctrl_y = y_e_vel;
				
			x_n_vel = -k1p*(cxyz_no_gps.x-cur_target_xyz.x)-k1d*(cvel_no_gps.x);
			y_e_vel = -k2p*(cxyz_no_gps.y-cur_target_xyz.y)-k2d*(cvel_no_gps.y);

			//TEST ONLY!!!!!!!
			del_ctrl_x_gps = x_n_vel - gps_ctrl_x;
			del_ctrl_y_gps = y_e_vel - gps_ctrl_y;
			printf("[NO GPS]DEL CTRL NO GPS X,Y--->%.8lf.\t%.8lf.\t\n",del_ctrl_x_gps,del_ctrl_y_gps);
		}

		//lower the x y control
		if(x_n_vel > MAX_CTRL_VEL_UPDOWN_WITH_IMAGE)
		{
			x_n_vel=MAX_CTRL_VEL_UPDOWN_WITH_IMAGE;
		}else if (x_n_vel < (-1.0) * MAX_CTRL_VEL_UPDOWN_WITH_IMAGE)
		{
			x_n_vel= (-1.0) * MAX_CTRL_VEL_UPDOWN_WITH_IMAGE;
		}

		if(y_e_vel > MAX_CTRL_VEL_UPDOWN_WITH_IMAGE)
		{
			y_e_vel=MAX_CTRL_VEL_UPDOWN_WITH_IMAGE;
		}else if (y_e_vel < (-1.0) * MAX_CTRL_VEL_UPDOWN_WITH_IMAGE)
		{
			y_e_vel= (-1.0) * MAX_CTRL_VEL_UPDOWN_WITH_IMAGE;
		}

		
		//printf("Ctrl_X, Y-->.\t.\t%f.\t%f\n", x_n_vel,y_e_vel);

		
		//x_n_vel = -k1p*(-1)*(offset_adjust.y)-k1d*(cvel.x);  	//camera y is to the south when DJI head focus north
		//y_e_vel = -k2p*(offset_adjust.x)-k2d*(cvel.y);	//camera x is to the east when DJI head focus north

		puser_ctrl_data->ctrl_flag=0x40;//´¹Ö±ËÙ¶È£¬Ë®Æ½ËÙ¶È£¬º½Ïò½Ç¶È¿ØÖÆÄ£Ê½
		puser_ctrl_data->roll_or_x = x_n_vel;			//x±±ÏòÆÚÍûËÙ¶È
		puser_ctrl_data->pitch_or_y = y_e_vel;		//y¶«ÏòÆÚÍûËÙ¶È

		if  (cpos.health_flag < GPS_HEALTH_GOOD && no_GPS_mode_on == 1)
		{		
			last_dis_to_mark=sqrt(pow((cxyz_no_gps.x- cur_target_xyz.x), 2)+pow((cxyz_no_gps.y-cur_target_xyz.y), 2));		
			
			if (last_dis_to_mark < HOVER_POINT_RANGE)
			{
				target_update = 0;
				no_GPS_mode_on = 0;
			}
		}
		else
		{
			last_dis_to_mark=sqrt(pow((cxyz.x- cur_target_xyz.x), 2)+pow((cxyz.y-cur_target_xyz.y), 2));
			
			if (last_dis_to_mark < HOVER_POINT_RANGE)
			{
				target_update = 0;
				no_GPS_mode_on = 0;
			}
		}
		
		
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

