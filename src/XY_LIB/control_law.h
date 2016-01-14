#ifndef __CONTROL_LAW_H__
#define __CONTROL_LAW_H__

#include <math.h>
#include <stdio.h>
#include "../DJI_LIB/DJI_Pro_App.h"
#include "wireless_debug.h"
#include "route.h"
#include "common/common.h"
#include "image_identify.h"

typedef struct{
	double x;
	double y;
	double z;
}XYZ;

typedef XYZ Center_xyz;

typedef struct{
	double roll_deg;
	double pitch_deg;
	double yaw_deg;
}Body_Angle;



int XY_Cal_Attitude_Ctrl_Data_Down_To_Height_WithVel_IMAGE(	api_vel_data_t cvel, 
														api_pos_data_t cpos,
														api_pos_data_t _focus_point,
														float target_vel, 
														float t_height,
														attitude_data_t *puser_ctrl_data,
														int *flag);

int XY_Cal_Attitude_Ctrl_Data_Up_To_Height_WithVel(	api_vel_data_t cvel, 
														api_pos_data_t cpos,
														api_pos_data_t _focus_point,
														float target_vel, 
														float t_height,
														attitude_data_t *puser_ctrl_data,
														int *flag);

int XY_Cal_Attitude_Ctrl_Data_UpDown_TO_Height(	api_vel_data_t cvel,
												api_pos_data_t cpos,
												api_pos_data_t _focus_point,
												float t_height,
												attitude_data_t *puser_ctrl_data,
												int *flag);

int XY_Cal_Attitude_Ctrl_Data_P2P(	api_vel_data_t cvel, 
									api_pos_data_t cpos, 
									float height, 
									Leg_Node *p_legn,
									attitude_data_t *puser_ctrl_data,
									int *flag);

int XY_Cal_Vel_Ctrl_Data_FP(api_vel_data_t cvel, api_pos_data_t cpos, api_pos_data_t spos, api_pos_data_t tpos, float height, attitude_data_t *puser_ctrl_data);
void geo2XYZ(api_pos_data_t pos, XYZ *pXYZ);
void init_g_origin_pos(api_pos_data_t *_g_origin_pos);
void XYZ2xyz(api_pos_data_t s_pos, XYZ pXYZ, Center_xyz*pxyz);
void QUA2ANGLE(api_quaternion_data_t cur_quaternion, Body_Angle *body_angle) ;

int XY_Get_and_Print_Image_Data(void);
void XY_Cal_Vel_Ctrl_Data_FP_With_IMAGE(api_vel_data_t cvel, api_pos_data_t cpos, float height, attitude_data_t *puser_ctrl_data,int *flag);
void XY_Cal_Vel_Ctrl_Data_Get_Down_With_IMAGE(api_vel_data_t cvel, api_pos_data_t cpos, attitude_data_t *puser_ctrl_data);




#endif

