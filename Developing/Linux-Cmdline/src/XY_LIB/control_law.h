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



int XY_Cal_Attitude_Ctrl_Data_UpDown(api_vel_data_t cvel, api_pos_data_t cpos, float t_height, attitude_data_t *puser_ctrl_data, int *flag);
int XY_Cal_Attitude_Ctrl_Data_P2P(api_vel_data_t cvel, api_pos_data_t cpos, float height, Leg_Node *p_legn, attitude_data_t *puser_ctrl_data, int *flag);
int XY_Cal_Vel_Ctrl_Data_FP(api_vel_data_t cvel, api_pos_data_t cpos, api_pos_data_t spos, api_pos_data_t tpos, float height, attitude_data_t *puser_ctrl_data);
void geo2XYZ(api_pos_data_t pos, XYZ *pXYZ);
void XYZ2xyz(api_pos_data_t s_pos, XYZ pXYZ, Center_xyz*pxyz);

#endif

