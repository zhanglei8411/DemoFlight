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


#define SAFETY_HEIGHT				(10.0)	//m
#define LOW_HEIGHT					(0.5)	//m



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
