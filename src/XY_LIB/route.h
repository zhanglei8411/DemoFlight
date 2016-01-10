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


#define TAKEOFF_HEIGHT_H_U1						(1.0)	//m
#define GO_UP_TO_HEIGHT_WTIH_IMAGE_H_U2			(10.0)	//m
#define GO_UP_TO_CRUICE_HEIGHT_H_U3				(40.0)	//m, 恒生小房子的安全高度
#define GO_DOWN_TO_HEIGHT_H_D3					(15.0)	//m
#define GO_DOWN_TO_HEIGHT_WITH_IMAGE_H_D2		(1.0)	//m
#define GO_DOWN_TO_HEIGHT_READY_TO_LAND_H_D1	(0.25)	//m

#define Ultra_DELTA_UNLOAD 						(0.045) //m， 0110试验实际安装数据
#define Ultra_DELTA_LOAD						(0.148) //m,章磊估计值，未测试

#define MODE_1_UP_TO_U2_IMAGE_VEL_1				1
#define MODE_2_UP_TO_U3							2
#define MODE_3_DOWN_TO_D3						3
#define MODE_4_DOWN_TO_D2_IMAGE_VEL_N05			4
#define MODE_5_DOWN_TO_D1						5


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

int temporary_init_route_list(void);
int setup_route_list_head_node(Leg_Node *head);
int insert_new_leg_into_route_list(Leg_Node *head, struct Leg leg);
Leg_Node *create_head_node(void);
int add_leg_node(Leg_Node *_head, struct Leg _leg);
void delete_leg_list(Leg_Node *_head);
int XY_Start_Route_Task_Thread(pthread_t *_thread);
void XY_Release_List_Resource(void);





#endif
