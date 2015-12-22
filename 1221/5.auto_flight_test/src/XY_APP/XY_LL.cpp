#include "XY_LL.h"


Link_Leg_Node *Create_Leg_Head(void)
{
	Link_Leg_Node *_head = NULL;

	_head = (Link_Leg_Node *)calloc(1, sizeof(Link_Leg_Node));
	if(_head == NULL)
	{
		return NULL;
	}
	_head->leg.leg_seq = 0;
	_head->next = NULL;

	return _head;
}

//在头结点后插入节点
int Add_Leg_Node(Link_Leg_Node *_head, Leg _leg)
{
	Link_Leg_Node *pcur, *pnew;
	int valid_seq;
	
	/* 1. 定位到尾节点 */
	pcur = _head;
	while(pcur->next)
	{
		pcur = pcur->next;
	}
	/* 2. 判断预插入的节点的数据是否合法 */
	valid_seq = pcur->leg.leg_seq+1;
	if(_leg.leg_seq != valid_seq)
	{
		return -1;
	}
	pnew = (Link_Leg_Node *)calloc(1, sizeof(Link_Leg_Node));
	if(pnew == NULL)
		return -1;
	
	Set_Leg_Seq(&(pnew->leg), _leg.leg_seq);
	Set_Leg_Num(&(pnew->leg), _leg.leg_num);
	Set_Leg_Start_Pos(&(pnew->leg), _leg.longti_s, _leg.lati_s, _leg.alti_s);
	Set_Leg_End_Pos(&(pnew->leg), _leg.longti_e, _leg.lati_e, _leg.alti_e);
	
	pcur->next = pnew;
	pnew->next = NULL;

	return 0;
}


void Delete_Leg_List(Link_Leg_Node *_head)
{
	Link_Leg_Node *p;

	while(_head->next)
	{
		p = _head;
		_head = _head->next;
		free(p);
	}
	free(_head);
	_head = p = NULL;	//防止出现野指针

}

