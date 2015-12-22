#ifndef __XY_CTRL_H__
#define __XY_CTRL_H__

#include <math.h>
#include <stdio.h>
#include "../DJI_LIB/DJI_Pro_App.h"
#include "XY_LL.h"
#include "XY_Debug.h"


int Cal_Attitude_Ctrl_Data_UpDown(api_vel_data_t cvel, api_pos_data_t cpos, float height, attitude_data_t *puser_ctrl_data, int *flag);
int Cal_Attitude_Ctrl_Data_P2P(api_vel_data_t cvel, api_pos_data_t cpos, float height, Link_Leg_Node *p_legn, attitude_data_t *puser_ctrl_data, int *flag);




#endif

