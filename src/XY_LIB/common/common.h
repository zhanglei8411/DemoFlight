#ifndef __COMMON_H__
#define __COMMON_H__


typedef struct{
	double _longti;
	double _lati;
	double _alti;
	float _height;
}position;

struct Leg
{
	unsigned char leg_seq;
	unsigned char leg_num;
	position start;
	position end;
	position current;
};

typedef struct _Leg_Node
{
	struct Leg leg;
	_Leg_Node *next;
}Leg_Node;

typedef struct
{
	volatile int obtained_control;
	volatile int activation;
	volatile int take_off;
	
}Aircraft_Status;




#endif
