#include "control_law.h"
#include "range.h"

#define DT 						(0.02)
#define HOVER_POINT_RANGE 		(0.1)
#define HOVER_VELOCITY_MIN 		(0.1)
#define TRANS_TO_HOVER_DIS 		(5.0) 

#define UPDOWN_CTRL_KP			(0.2)
#define HEIGHT_CTRL_DELTA		(0.5)

extern struct debug_info debug_package;
extern pthread_mutex_t debug_mutex;
extern pthread_cond_t debug_cond;


/* ´óµØ->ÇòĞÄ */
void geo2XYZ(api_pos_data_t pos, XYZ *pXYZ)
{
	double a = 6378137;				//aÎªÍÖÇòµÄ³¤°ëÖá:a=6378.137km
	double b = 6356752.3141;			//bÎªÍÖÇòµÄ¶Ì°ëÖá:a=6356.7523141km
	double H = pos.alti;	//delete"+a"by zhanglei 0108
	double e = sqrt(1-pow(b ,2)/pow(a ,2));//eÎªÍÖÇòµÄµÚÒ»Æ«ĞÄÂÊ  
	double B = pos.lati;
	double L = pos.longti;
	double W = sqrt(1-pow(e ,2)*pow(sin(B) ,2));
	double N = a/W; //NÎªÍÖÇòµÄÃ®ÓÏÈ¦ÇúÂÊ°ë¾¶ 
	
	pXYZ->x = (N+H)*cos(B)*cos(L);
	pXYZ->y = (N+H)*cos(B)*sin(L);
	pXYZ->z = (N*(1-pow(e ,2))+H)*sin(B);
}

/*ÇòĞÄ->Õ¾ĞÄ add by zhanglei, 20151224*/
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


/*ÒÔÖ¸¶¨ËÙ¶Èµ½´ïÖ¸¶¨¸ß¶ÈÉÏÉıÏÂ½µ¶Î¿ØÖÆ*/

int XY_Cal_Attitude_Ctrl_Data_UpDown_To_H_WithVel(api_vel_data_t cvel, api_pos_data_t cpos, float target_vel, float t_height, attitude_data_t *puser_ctrl_data, int *flag)
{
	
	static api_pos_data_t epos;	
	double kp_z=UPDOWN_CTRL_KP;  //Up down cotrol factor, add by zhanglei 1225

	/*¼ÇÂ¼µ±Ç°Î»ÖÃ£¬ÎªÄ¿±êË®Æ½Î»ÖÃ*/
	static int count = 0;	
	if(count == 0)
	{
		epos.longti=cpos.longti;
		epos.lati=cpos.lati;
		epos.alti=cpos.alti;
		epos.height=cpos.height;
		count++;
	}
	
	//¶¨µã¿ØÖÆ£¬µ«²»×ö¸ß¶È¿ØÖÆ
	XY_Cal_Vel_Ctrl_Data_FP(cvel, cpos, epos, epos, epos.height, puser_ctrl_data); //´Ë´¦epos.heightÉèÖÃµÄ¸ß¶ÈÖµÃ»ÓĞÒâÒå£¬²»×ö¸ß¶È¿ØÖÆ

	puser_ctrl_data->thr_z = kp_z * (t_height - cpos.height)+target_vel;   //¿ØÖÆµÄÊÇ´¹Ö±ËÙ¶È

	//addÏŞ·ù´úÂë
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

	XY_Debug_Sprintf(0, "\n[Height] %.8f.\n", cpos.height);
	
}


/*µ½´ïÖ¸¶¨¸ß¶ÈµÄÉÏÉıÏÂ½µ¶Î¿ØÖÆ*/
int XY_Cal_Attitude_Ctrl_Data_UpDown_TO_H(api_vel_data_t cvel, api_pos_data_t cpos, float t_height, attitude_data_t *puser_ctrl_data, int *flag)
{
	static api_pos_data_t epos;	
	double kp_z=UPDOWN_CTRL_KP;  //Up down cotrol factor, add by zhanglei 1225
	
	/*¼ÇÂ¼µ±Ç°Î»ÖÃ£¬ÎªÄ¿±êË®Æ½Î»ÖÃ*/
	static int count = 0;	
	if(count == 0)
	{
		epos.longti=cpos.longti;
		epos.lati=cpos.lati;
		epos.alti=cpos.alti;
		epos.height=cpos.height;
		count++;
	}
	
	//¶¨µã¿ØÖÆ£¬µ«²»×ö¸ß¶È¿ØÖÆ
	XY_Cal_Vel_Ctrl_Data_FP(cvel, cpos, epos, epos, epos.height, puser_ctrl_data); //´Ë´¦epos.heightÉèÖÃµÄ¸ß¶ÈÖµÃ»ÓĞÒâÒå£¬²»×ö¸ß¶È¿ØÖÆ

	puser_ctrl_data->thr_z = kp_z * (t_height - cpos.height);   //æ§åˆ¶çš„æ˜¯å‚ç›´é€Ÿåº¦

	//addÏŞ·ù´úÂë
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

	XY_Debug_Sprintf(0, "\n[Height] %.8f.\n", cpos.height);

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


/*	//è·Ÿè¸ªç‚¹æ§åˆ¶å‚æ•°, last modified by zhanglei, 1222
	k1d=0.5;
	k1p=1;		//1222 by zhanglei è°ƒæ•´å¢åŠ 10å€ï¼Œé£è¡Œæµ‹è¯•okï¼Œä¹‹å‰ä¸ºä»¿çœŸokçš„å‚æ•°0.1
	k2d=0.5;
	k2p=1;		//1222 by zhanglei è°ƒæ•´å¢åŠ 10å€ï¼Œé£è¡Œæµ‹è¯•okï¼Œä¹‹å‰ä¸ºä»¿çœŸokçš„å‚æ•°0.1
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

	//ä»å¤§åœ°åæ ‡ç³»è½¬æ¢åˆ°çƒå¿ƒåæ ‡ç³»
	geo2XYZ(epos, &eXYZ);
	geo2XYZ(cpos, &cXYZ);
	//´ÓÇòĞÄ×ø±êÏµ×ªµ½Õ¾ĞÄ×ø±êÏµadd by zhanglei, 1225;
	XYZ2xyz(spos, eXYZ, &exyz);
	XYZ2xyz(spos, cXYZ, &cxyz);


	//ç«™å¿ƒåæ ‡ç³»çš„æ§åˆ¶å‚æ•°add by zhanglei, 1225;
	k1d=0.5;
	k1p=1;		
	k2d=0.5;
	k2p=1;	

	//ç«™å¿ƒåæ ‡ç³»ä¸‹è¿›è¡Œå§¿æ€æ§åˆ¶add by zhanglei, 1225;
	thetaC= k1p*(cxyz.x-exyz.x)+k1d*cvel.x;
	phiC=-k2p*(cxyz.y-exyz.y)-k2d*cvel.y;

/*	
	//åœ¨çƒå¿ƒåæ ‡ç³»ä¸‹è·Ÿè¸ªç‚¹å§¿æ€æ§åˆ¶, add by zhanglei, 1224; simulated 1225am ok by zl.
    thetaC = k1p*(cXYZ.z-eXYZ.z) + k1d*cvel.x;//æœŸæœ›çš„ä¿¯ä»°è§’ XYZçƒå¿ƒåæ ‡ç³»ï¼ŒZè½´North+,+thetaC_pitchäº§ç”ŸSouthé€Ÿé€’
	phiC = k2p*(cXYZ.x-eXYZ.x) - k2d*cvel.y;//æœŸæœ›çš„æ»šè½¬è§’ XYZçƒå¿ƒåæ ‡ç³»ï¼ŒXè½´West+, +phiC_rolläº§ç”ŸEasté€Ÿåº¦
*/

	puser_ctrl_data->ctrl_flag=0x00;//å‚ç›´é€Ÿåº¦ï¼Œæ°´å¹³å§¿æ€ï¼Œèˆªå‘è§’åº¦æ§åˆ¶æ¨¡å¼
	puser_ctrl_data->roll_or_x = phiC;			//æ»šè½¬è§’.æœºä½“xè½´ã€‚æŒ‰ç…§ç›®å‰ Groundåæ ‡ç³»ï¼Œäº§ç”Ÿyè½´é€Ÿåº¦
	puser_ctrl_data->pitch_or_y = thetaC;		//ä¿¯ä»°è§’.æœºä½“yè½´ã€‚æŒ‰ç…§ç›®å‰ Groundåæ ‡ç³»ï¼Œäº§ç”Ÿ-xè½´é€Ÿåº¦
	puser_ctrl_data->thr_z =  height - cpos.height;   // é«˜åº¦å•ä½è´Ÿåé¦ˆæ§åˆ¶ï¼ŒåæœŸå¯è°ƒæ•´åé¦ˆç³»æ•°ä¼˜åŒ–æ€§èƒ½ -z 
	puser_ctrl_data->yaw = 0;

//	last_distance=sqrt(pow((-1)*(cXYZ.x- eXYZ.x), 2)+pow((cXYZ.z-eXYZ.z), 2));//Xè½´åœ¨ä¸œåŠçƒå‘è¥¿ä¸ºæ­£ï¼Œåœ¨xè½´å¢åŠ è´Ÿå·

	last_distance_xyz=sqrt(pow((cxyz.x- exyz.x), 2)+pow((cxyz.y-exyz.y), 2));

#if 0
		printf("Dis--> X:%.8lf, Y:%.8lf\n",(cxyz.x- exyz.x), (cxyz.y-exyz.y));//xè½´åœ¨ä¸œåŠçƒå‘è¥¿ä¸ºæ­£ï¼Œåœ¨xè½´å¢åŠ è´Ÿå·
#endif
	
	if(last_distance_xyz < TRANS_TO_HOVER_DIS)
	{
		*flag = XY_Cal_Vel_Ctrl_Data_FP(cvel, cpos, spos, epos, height, puser_ctrl_data);
		count = 0;
	}


	XY_Debug_Sprintf(0, "\n[Height] %.8f.\n", cpos.height);
	XY_Debug_Sprintf(1, "[XYZ] %.8lf, %.8lf.\n", (cxyz.x- exyz.x), (cxyz.y-exyz.y));
	
}


/*=====¶¨µãĞüÍ£¿ØÖÆ============*/
/*Author: zhanglei
/*Create:2015-12-23
/*Last Modify: 2015-12-24 by zhanglei
/*Input: µ±Ç°ËÙ¶È£¬µ±Ç°¾­Î³¶È
/*		Õ¾ĞÄ¾­Î³¶È£¬Ä¿±ê¾­Î³¶È
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

	//´Ó´óµØ×ø±êÏµ×ª»»µ½ÇòĞÄ×ø±êÏµ
	geo2XYZ(tpos, &tXYZ);
	geo2XYZ(cpos,&cXYZ);

	//´ÓÇòĞÄ×ø±êÏµ×ª»»µ½Õ¾ĞÄ×ø±êÏµ
	XYZ2xyz(spos, cXYZ, &cxyz);
	XYZ2xyz(spos, tXYZ, &txyz);

	//1225ÏÂÎç·ÂÕæ½á¹û²ÎÊı£¬by zhanglei ok, 1227·ÉĞĞ²ÎÊıok
	k1d=0.05;
	k1p=0.4;	
	k2d=0.05;
	k2p=0.4;	

	x_n_vel = -k1p*(cxyz.x-txyz.x)-k1d*(cvel.x);//xyzÕ¾ĞÄ×ø±êÏµ£¬xÖá±±ÏòÕı
	y_e_vel = -k2p*(cxyz.y-txyz.y)-k2d*(cvel.y);//xyzÕ¾ĞÄ×ø±êÏµ£¬yÖá¶«ÏòÕı

	puser_ctrl_data->ctrl_flag=0x40;//´¹Ö±ËÙ¶È£¬Ë®Æ½ËÙ¶È£¬º½Ïò½Ç¶È¿ØÖÆÄ£Ê½
	puser_ctrl_data->roll_or_x = x_n_vel;			//x±±ÏòÆÚÍûËÙ¶È
	puser_ctrl_data->pitch_or_y = y_e_vel;		//y¶«ÏòÆÚÍûËÙ¶È
	puser_ctrl_data->thr_z =  height - cpos.height;   // ¸ß¶Èµ¥Î»¸º·´À¡¿ØÖÆ£¬ºóÆÚ¿Éµ÷Õû·´À¡ÏµÊıÓÅ»¯ĞÔÄÜ 

	last_distance_xyz=sqrt(pow((cxyz.x- txyz.x), 2)+pow((cxyz.y-txyz.y), 2));

#if 0
	printf("HoverDis--> X:%.8lf, Y:%.8lf\n",(cxyz.x- txyz.x),(cxyz.y-txyz.y));
#endif

	if(last_distance_xyz < HOVER_POINT_RANGE)
	{
		printf("HoverVel--> X:%.8lf, Y:%.8lf\n",cvel.x,cvel.y);
		puser_ctrl_data->roll_or_x = 0;			//x±±ÏòÆÚÍûËÙ¶È
		puser_ctrl_data->pitch_or_y = 0;		//y¶«ÏòÆÚÍûËÙ¶È		

		last_velocity=sqrt(cvel.x*cvel.x+cvel.y*cvel.y);

		if(last_velocity < HOVER_VELOCITY_MIN)
		{
			
			//XY_Cal_Vel_Ctrl_Data_Image(cpos.height);	//offset+ËÄÔªÊı+¸ß¶È(cpos.height)
			return 1;
		}
	}
	return 0;
	
}

/*
	»ñÈ¡³¬Éù²¨Êı¾İµÄ·½Ê½

	float data;
	if(XY_Get_Ultra_Data(&data) == 0)
   	{
		»ñÈ¡µÄÊı¾İÓĞĞ§
		¿ÉÒÔ¶Ôdata½øĞĞ·ÖÎö
   	}

	»ñÈ¡×ËÌ¬ËÄÔªÊı
	api_quaternion_data_t cur_quaternion;
	DJI_Pro_Get_Quaternion(&cur_quaternion);

	»ñÈ¡Í¼Ïñoffset
	Offset offset;
	if( XY_Get_Offset_Data(&offset) == 0)	//·µ»Ø0±íÊ¾Êı¾İÓĞĞ§
			printf("Get Offset - x:%.4f, y:%.4f, z:%.3f\n", offset.x, offset.y, offset.z);

	´ò¿ª/¹Ø±ÕÉãÏñÍ·
	XY_Start_Capture();
	XY_Stop_Capture();

 */

