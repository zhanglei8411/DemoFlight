#ifndef __ROUTE_H__
#define __ROUTE_H__

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include "../DJI_LIB/DJI_Pro_App.h"
#include "control_law.h"
#include "wireless_debug.h"
#include "common/common.h"
#include "control_steer.h"


#define DELIVER_MAX_VAL_UP_TO_H2								(0.75)	//m/s
#define DELIVER_MAX_VAL_UP_TO_H3								(1.5)
#define DELIVER_MAX_VAL_DOWN_TO_H1								(1.5)
#define DELIVER_MAX_VAL_DOWN_TO_H2								(0.35)
#define DELIVER_MAX_VAL_DOWN_TO_H3								(0.25)



#define DELIVER_HEIGHT_OF_UPH2										(10.0)	//m
#define DELIVER_HEIGHT_OF_UPH3										(35.0)
#define DELIVER_HEIGHT_OF_DOWNH1									(25.0)
#define DELIVER_HEIGHT_OF_DOWNH2									(1.0)
#define DELIVER_HEIGHT_OF_DOWNH3									(0.25)

#define GOBACK_HEIGHT_OF_UPH2										DELIVER_HEIGHT_OF_UPH2
#define GOBACK_HEIGHT_OF_UPH3										DELIVER_HEIGHT_OF_UPH3
#define GOBACK_HEIGHT_OF_DOWNH1										DELIVER_HEIGHT_OF_DOWNH1
#define GOBACK_HEIGHT_OF_DOWNH2										DELIVER_HEIGHT_OF_DOWNH2
#define GOBACK_HEIGHT_OF_DOWNH3										DELIVER_HEIGHT_OF_DOWNH3



#define DELIVER_THRESHOLD_OF_UP_TO_H2_OUT						(1.0)		
#define DELIVER_THRESHOLD_OF_UP_TO_H3_OUT						(1.0)	
#define DELIVER_THRESHOLD_OF_DOWN_TO_H1_OUT						(1.0)	
#define DELIVER_THRESHOLD_OF_DOWN_TO_H2_OUT						(0.25)
#define DELIVER_THRESHOLD_OF_DOWN_TO_H3_OUT						(0.15)



#define DELIVER_UP_TO_H2_KPZ									(0.2)
#define DELIVER_UP_TO_H3_KPZ									(0.2)
#define DELIVER_DOWN_TO_H1_KPZ									(0.2)
#define DELIVER_DOWN_TO_H2_KPZ									(0.2)
#define DELIVER_DOWN_TO_H3_KPZ									(0.2)






#define TAKEOFF_HEIGHT_H_U1						(1.0)	//m
#define GO_UP_TO_HEIGHT_WTIH_IMAGE_H_U2			(10.0)	//m
#define GO_UP_TO_CRUICE_HEIGHT_H_U3				(35.0)	//m, ºãÉúÐ¡·¿×ÓµÄ°²È«¸ß¶È
#define GO_DOWN_TO_HEIGHT_H_D3					(25.0)	//m
#define GO_DOWN_TO_HEIGHT_WITH_IMAGE_H_D2		(1.0)	//m
#define GO_DOWN_TO_HEIGHT_READY_TO_LAND_H_D1	(0.25)	//m

#define UP_TO_HEIGHT_WITH_VEL_AT_END			(1.5)	//m, up is positive +, suggest be same as the vel in next stage
#define DOWN_TO_HEIGHT_WITH_VEL_AT_END			(-0.5)	//m, down is negtive -, suggest be same as the vel in next stage

#define Ultra_DELTA_UNLOAD 						(0.045) //m£¬ 0110ÊÔÑéÊµ¼Ê°²×°Êý¾Ý
#define Ultra_DELTA_LOAD						(0.148) //m,ÕÂÀÚ¹À¼ÆÖµ£¬Î´²âÊÔ

#define MODE_1_UP_TO_U2_IMAGE_VEL_1				1
#define MODE_2_UP_TO_U3							2
#define MODE_3_DOWN_TO_D3						3
#define MODE_4_DOWN_TO_D2_IMAGE_VEL_N05			4
#define MODE_5_DOWN_TO_D1						5
#define MODE_T1_TEST_HOVER_WITH_IMAGE			6


#define XYI_TERRACE_LONGTI						(2.09443030)//Google Earth
#define XYI_TERRACE_LATI						(0.52849836)//Google Earth
#define XYI_TERRACE_ALTI						(0.0)

#define ORIGIN_IN_HENGSHENG_LONGTI				(2.094421665) //Google EarthÌØÕ÷µã×ø±ê²ÎÊý,»¡¶È.9
#define ORIGIN_IN_HENGSHENG_LATI				(0.528485654) //Google EarthÌØÕ÷µã×ø±ê²ÎÊý,»¡¶È.9
#define ORIGIN_IN_HENGSHENG_ALTI				(0.0)

/*
#define DELTA_X_M_GOOGLEEARTH					(3.611858)   //m base the data in test on 0113-2
#define DELTA_Y_M_GOOGLEEARTH					(0.243988)   //m	
#define DELTA_Z_M_GOOGLEEARTH					(-110.209)   //m	
*/

/* 1-15 test */
#define DELTA_X_M_GOOGLEEARTH					(-6.302767)   //m base the data in test on 0113-2
#define DELTA_Y_M_GOOGLEEARTH					(-0.504967)   //m	
#define DELTA_Z_M_GOOGLEEARTH					(-110.209)   //m	


#define GPS_DELTA_LONGTI	(-0.000081)					//»¡¶È£¬Ïà¶ÔÓÚGoogle map
#define GPS_DELTA_LATI		(0.000044)
#define TARGET_LONTI_FROM_GOOGLE	2.094503592			//ÓðÃ«Çò³¡Æð·Éµã
#define TARGET_LATI_FROM_GOOGLE		0.528442797
#define TARGET_ALTI_FROM_DJI_TEST	-139.26440430	


inline void set_leg_seq(struct Leg *_pleg, int _leg_seq)
{
	_pleg->leg_seq = _leg_seq;
}

inline void set_leg_num(struct Leg *_pleg, int _leg_num)
{
	_pleg->leg_num = _leg_num;
}

inline void set_pos(position *_pos, double _longti, double _lati, double _alti)
{
	_pos->_longti = _longti;
	_pos->_lati = _lati;
	_pos->_alti = _alti;
}

inline void set_leg_start_pos(struct Leg *_pleg, double _longti, double _lati, double _alti)
{
	set_pos(&_pleg->start, _longti, _lati, _alti);
}

inline void set_leg_end_pos(struct Leg *_pleg, double _longti, double _lati, double _alti)
{
	set_pos(&_pleg->end, _longti, _lati, _alti);
}

inline void set_leg_cur_pos(struct Leg *_pleg, double _longti, double _lati, double _alti)
{
	set_pos(&_pleg->current, _longti, _lati, _alti);
}


int setup_route_list_head_node(Leg_Node *head);
int insert_new_leg_into_route_list(Leg_Node *head, struct Leg leg);
Leg_Node *create_head_node(void);
int add_leg_node(Leg_Node *_head, struct Leg _leg);
void delete_leg_list(Leg_Node *_head);
void XY_Release_List_Resource(Leg_Node *_head);
int update_cur_legn_data(double _longti, double _lati);
int temporary_init_deliver_route_list(void);
int temporary_init_goback_route_list(void);


int XY_Drone_Execute_Task(void);
int XY_Drone_Deliver_Task(void);
int XY_Drone_Goback_Task(void);
static void *drone_deliver_task_thread_func(void * arg);
static void *drone_deliver_up_thread_func(void * arg);
static void *drone_deliver_p2p_thread_func(void * arg);
static void *drone_deliver_down_thread_func(void * arg);
int XY_Ctrl_Drone_P2P_With_FP_COMMON(float _p2p_height, int _goback);
int XY_Ctrl_Drone_To_Assign_Height_Has_MaxVal_And_FP_DELIVER(float _max_vel, float _t_height, float _threshold, double _kp_z);
int XY_Ctrl_Drone_Spot_Hover_And_Find_Put_Point_DELIVER(void);
int XY_Ctrl_Drone_Down_Has_NoGPS_Mode_And_Approach_Put_Point_DELIVER(float _max_vel, float _t_height, float _threshold, double _kp_z);
int XY_Ctrl_Drone_To_Spot_Hover_And_Put_DELIVER(void);

static void *drone_goback_task_thread_func(void * arg);
static void *drone_goback_up_thread_func(void * arg);
static void *drone_goback_p2p_thread_func(void * arg);
static void *drone_goback_down_thread_func(void * arg);














#endif
