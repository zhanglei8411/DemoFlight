#ifndef __CONTROL_LAW_H__
#define __CONTROL_LAW_H__

#include <math.h>
#include <stdio.h>
#include "../DJI_LIB/DJI_Pro_App.h"
#include "wireless_debug.h"
#include "route.h"
#include "common/common.h"
#include "image_identify.h"
#include "ultra_filter.h"

#define DT 								(0.02)	//the period, for integration
#define PI								(3.1415926)

/*GPS mode*/
#define VEL_CTRL_MODE                   1
#define ATTI_CTRL_MODE                  2
#define GPS_OK_FOR_USE					4     //>= this value, xy vel use gps 
#define GPS_VERY_GOOD					5      //for test, normal is 5 for DJI M100
#define CTRL_MODE_CHANGE_GPS            3      //if higher than this, use vel ctrl；if gps health from inner controller is bigger than this value, the gps vel can be use

/*move xy para*/
#define HOVER_POINT_RANGE 				(0.1)	//FP use
#define TRANS_TO_HOVER_DIS 				(13.0) 	// for P2P trans to FP, can del
#define P2P_MAX_VEL_N_E					(3.0)

/*ultra para for z*/
#define ULTRA_INSTALL_HEIGHT            (0.135) //install diff of ultrasonic equip, 0.135m
#define HEIGHT_TO_USE_ULTRA				(3.5)	//0221 (5.0 to 3.5)
#define DEPTH_ULTRA_Z_VEL_CAL			6		//the depth of the data queue of ultra data which is used to cal the z velocity, max is 10
#define SEPT_TIMES_FOR_CAL_Z_VEL		4		//the time of period of ultra data between two data to cal the velocity

/*image para for xy*/
#define DIS_DIFF_WITH_MARK				(0.10)	//image FP & image down use
#define DIS_DIFF_WITH_MARK_HOVER		(1.5)	//hover to get close to the image, due to the high height, the dis is large

#define MAX_EACH_DIS_IMAGE_GET_CLOSE	(3.0)	//each time the image target dis limit
#define MAX_CTRL_VEL_UPDOWN_WITH_IMAGE	(0.35)	//just for x,y control, change from 0.5 by zhanglei night 0114
#define MAX_CTRL_ATTI_UPDOWN_WITH_IMAGE	(5.0)	//just for x,y control, add by zhanglei 0310 night

#define CAM_INSTALL_DELTA_X				(0.0)	//m,add to offset camera x, down to see drone, x is right 
#define CAM_INSTALL_DELTA_Y				(0.218)	//m,add to offset camera y, down to see drone, y is down, the drone center is y+

#define CAM_INSTALL_DELTA_ROLL          (3.0)      //when the camera head rotation right, +, add the angle
#define CAM_INSTALL_DELTA_PITCH         (1.8)      //when the camera head rotation up, +, add the angle

#define DEPTH_IMAGE_XY_CAL				5		//the depth of the data queue of image xy data which is used to cal the xy velocity, max is 5
#define SEPT_TIMES_FOR_CAL_XY_VEL		3		//the times of period of image data between two data to cal the velocity

#define MAX_CAM_DIFF_ADJUST				(1.0)	//use for offset_adjust cal

#define UP_DOWN_YAW_REDUCE				(0.9)	//"XY_Ctrl_Drone_To_Assign_Height_Has_MaxVel_And_FP_DELIVER" use

#define CAM_DIFFRENCE_RAD               (PI / 2) // CAM X-AXIS ANGLE FROM PLANE HEAD;

typedef XYZ Center_xyz;

typedef struct{
	double roll_deg;
	double pitch_deg;
	double yaw_deg;
}Body_Angle;


void geo2XYZ(api_pos_data_t pos, XYZ *pXYZ);
void init_g_origin_pos(api_pos_data_t *_g_origin_pos);
void XYZ2xyz(api_pos_data_t s_pos, XYZ pXYZ, Center_xyz*pxyz);
void QUA2ANGLE(api_quaternion_data_t cur_quaternion, Body_Angle *body_angle) ;
void Rotation_XY(double *ground_offet_x, double *ground_offet_y, double cam_offset_x, double cam_offset_y, float yaw_angle);

int XY_Ctrl_Drone_Circle(float _p2p_height);//zzy0321
int XY_Ctrl_Drone_Find_Point(void);


int XY_Ctrl_Drone_P2P_With_FP_COMMON(float _p2p_height, int _goback);
int XY_Ctrl_Drone_To_Assign_Height_Has_MaxVel_And_FP_DELIVER(float _max_vel, float _min_vel, float _t_height, float _threshold, float _t_yaw, float _yaw_threshold, double _kp_z);
int XY_Ctrl_Drone_Spot_Hover_And_Find_Put_Point_DELIVER(void);
int XY_Ctrl_Drone_Up_Has_NoGPS_Mode_And_Approach_Put_Point_DELIVER(float _max_vel, float _min_vel, float _t_height, float _threshold, double _kp_z);
int XY_Ctrl_Drone_Down_Has_NoGPS_Mode_And_Approach_Put_Point_DELIVER(float _max_vel, float _min_vel, float _t_height, float _threshold, double _kp_z);
int XY_Ctrl_Drone_Up_Has_NoGPS_Mode_And_Approach_Put_Point_GOBACK(float _max_vel, float _min_vel, float _t_height, float _threshold, double _kp_z);
int XY_Ctrl_Drone_Down_Has_NoGPS_Mode_And_Approach_Put_Point_GOBACK(float _max_vel, float _min_vel, float _t_height, float _threshold, double _kp_z);
int XY_Ctrl_Drone_To_Spot_Hover_And_Put_DELIVER(void);



#endif

