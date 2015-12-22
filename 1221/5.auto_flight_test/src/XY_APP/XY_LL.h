#ifndef __XY_LL_H__
#define __XY_LL_H__


#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>


typedef struct 
{
	unsigned char leg_seq;
	unsigned char leg_num;
	double longti_s;
	double lati_s;
	double alti_s;
	double longti_e;
	double lati_e;
	double alti_e;
	double longti_cur;
	double lati_cur;
	double alti_cur;
}Leg;

typedef struct _Link_Leg_Node
{
	Leg leg;
	_Link_Leg_Node *next;
}Link_Leg_Node;

#if 0
inline void Set_Leg_Seq(Leg *_pleg, int _leg_seq);
inline void Set_Leg_Num(Leg *_pleg, int _leg_num);
inline void Set_Leg_Start_Pos(Leg *_pleg, double _longti, double _lati);
inline void Set_Leg_End_Pos(Leg *_pleg, double _longti, double _lati);
inline void Set_Leg_Cur_Pos(Leg *_pleg, double _longti, double _lati);
#endif

inline void Set_Leg_Seq(Leg *_pleg, int _leg_seq)
{
	_pleg->leg_seq = _leg_seq;
}

inline void Set_Leg_Num(Leg *_pleg, int _leg_num)
{
	_pleg->leg_num = _leg_num;
}

inline void Set_Leg_Start_Pos(Leg *_pleg, double _longti, double _lati, double _alti)
{
	_pleg->longti_s = _longti;
	_pleg->lati_s = _lati;
	_pleg->alti_s = _alti;
}

inline void Set_Leg_End_Pos(Leg *_pleg, double _longti, double _lati, double _alti)
{
	_pleg->longti_e = _longti;
	_pleg->lati_e = _lati;
	_pleg->alti_e = _alti;
}

inline void Set_Leg_Cur_Pos(Leg *_pleg, double _longti, double _lati, double _alti)
{
	_pleg->longti_cur = _longti;
	_pleg->lati_cur = _lati;
	_pleg->alti_cur = _alti;
}



Link_Leg_Node *Create_Leg_Head(void);
int Add_Leg_Node(Link_Leg_Node *_head, Leg _leg);
void Delete_Leg_List(Link_Leg_Node *_head);



#endif
