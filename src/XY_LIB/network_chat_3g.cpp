/*
 * Description	: Chat by TCP
 * Author		: Zibo Zhou
 * Time			: 2016/1/6
 */

#include "network_chat_3g.h"
#include "thread_common_op.h"
#include "../DJI_LIB/DJI_Pro_Codec.h"

int sock_fd = -1;
int chat_active = 1;
NETFilter net_data = { 0 };


int XY_Chat_Setup(void)
{
	if( create_link_in_tcp() < 0)
	{
		return -1;
	}
	if(XY_Create_Thread(chat_recv_thread_func, NULL, THREADS_CHATR, -1, SCHED_RR, 90) < 0)
	{
		printf("Create Debug And Send Thread Error.\n");
		return -1;
	}
}


int create_socket(int *_sock_fd)
{
	if( (*_sock_fd = socket(AF_INET, SOCK_STREAM, 0) ) == -1)
	{
		perror("TCP socket");
		return -1;
	}
	return 0;
}

void setup_addr(struct sockaddr_in *_addr, unsigned short _port, const char *_cp)
{
	_addr->sin_family = AF_INET;
	_addr->sin_port = htons(_port);
	_addr->sin_addr.s_addr = inet_addr(_cp);
	bzero(&(_addr->sin_zero), 8);
}

int connect_to_server(int _sock_fd)
{
	struct sockaddr_in server_addr;
	
	setup_addr(&server_addr, SERVER_PORT, SERVER_IP);

	if( connect(_sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr) ) == -1)
	{
		perror("TCP connect");
		return -1;
	}

	return 0;
}

int create_link_in_tcp(void)
{
	if(create_socket(&sock_fd) < 0)
	{
		return -1;
	}

	if(connect_to_server(sock_fd) < 0)
	{
		close(sock_fd);
		return -1;
	}

	return 0;
}


static void *chat_recv_thread_func(void * arg)
{	
	char _rbuf[FRAME_SIZE];
	int ret = 0;
	
	while(1)
	{
		memset(_rbuf, 0, strlen((const char *)_rbuf));
		ret = recv(sock_fd, _rbuf, FRAME_SIZE, 0);
		if(ret == -1)
		{
			perror("TCP recv");
			continue;
		}
		else if(ret == 0)
		{
			printf("ERROR: Chat is inactive.\n");
			chat_active = 0;
			/* 调用重连函数 */

			continue;
		}
		//printf("%dbytes: %s\n", ret, _rbuf);
		
		for(int i = 0; i < ret; i ++)
		{
			/* 未完成, 未测试 */
			tcp_byte_handle(_rbuf[i]);
		}
		
		
	}
	pthread_exit(NULL);
}

void tcp_stream_prepare(NETFilter* p_filter)
{
	unsigned int bytes_to_move = sizeof(NETHeader) - 1;
	unsigned int index_of_move = p_filter->recv_index - bytes_to_move;

	memmove(p_filter->comm_recv_buf, p_filter->comm_recv_buf + index_of_move, bytes_to_move);
	memset(p_filter->comm_recv_buf + bytes_to_move, 0, index_of_move);
	p_filter->recv_index = bytes_to_move;
}


void Net_Link_Recv_Hook(NETHeader *header)
{
	if(header->is_ack == 1)
	{
	
	}
	else
	{
		
	}
}

typedef void(*net_filter_hook)(NETHeader* p_head);
net_filter_hook net_hook = (net_filter_hook)Net_Link_Recv_Hook;

/* analyze data */
void tcp_call_data_app(NETFilter* p_filter)
{
	// pass current data to handler
	NETHeader* p_head = (NETHeader*)p_filter->comm_recv_buf;

	if (net_hook)
	{
		net_hook((NETHeader*)p_filter->comm_recv_buf);
	}

	tcp_stream_prepare(p_filter);
}


void tcp_stream_shift_data(NETFilter* p_filter)
{
	if (p_filter->recv_index)
	{
		p_filter->recv_index--;
		if (p_filter->recv_index)
		{
			memmove(p_filter->comm_recv_buf, p_filter->comm_recv_buf + 1, p_filter->recv_index);
		}
	}
}


void tcp_stream_store_data(NETFilter* p_filter, unsigned char in_data)
{
	if (p_filter->recv_index < _NET_MAX_RECV_SIZE)
	{
		p_filter->comm_recv_buf[p_filter->recv_index] = in_data;
		p_filter->recv_index++;
	}
	else
	{
		// Error, buffer overflow! Just clear and continue.
		memset(p_filter->comm_recv_buf, 0, p_filter->recv_index);
		p_filter->recv_index = 0;
	}
}

void tcp_stream_update_reuse_part(NETFilter* p_filter)
{
	unsigned char* p_buf = p_filter->comm_recv_buf;
	unsigned short bytes_to_move = p_filter->recv_index - sizeof(NETHeader);
	unsigned char* p_src = p_buf + sizeof(NETHeader);

	unsigned short n_dest_index = p_filter->reuse_index - bytes_to_move;
	unsigned char* p_dest = p_buf + n_dest_index;

	memmove(p_dest, p_src, bytes_to_move);

	p_filter->recv_index = sizeof(NETHeader);
	tcp_stream_shift_data(p_filter);
	p_filter->reuse_index = n_dest_index;
	p_filter->reuse_count++;
}


void tcp_stream_verify_data(NETFilter* p_filter)
{
	NETHeader* p_head = (NETHeader*)(p_filter->comm_recv_buf);
	if (_SDK_CALC_CRC_TAIL(p_head, p_head->length) == 0)
	{
		tcp_call_data_app(p_filter);
	}
	else
	{
		//data crc fail
		tcp_stream_update_reuse_part(p_filter);
	}
}


void tcp_stream_verify_head(NETFilter* p_filter)
{
	NETHeader* p_head = (NETHeader*)(p_filter->comm_recv_buf);

	if ((p_head->sof == _NET_SOF) &&
		(p_head->length <= _NET_MAX_RECV_SIZE) &&
		(p_head->reversed0 == 0) &&
		(_SDK_CALC_CRC_HEAD(p_head, sizeof(NETHeader)) == 0)
		)
	{
		// check if this head is a ack or simple package
		if (p_head->length == sizeof(NETHeader))
		{
			//tcp_call_data_app(p_filter);
		}
	}
	else
	{
		tcp_stream_shift_data(p_filter);
	}
}

void tcp_check_stream_state(NETFilter* p_filter)
{
	NETHeader* p_head = (NETHeader*)(p_filter->comm_recv_buf);

	if (p_filter->recv_index < sizeof(NETHeader))
	{
		// Continue receive data, nothing to do

	}
	else if (p_filter->recv_index == sizeof(NETHeader))
	{
		// recv a full-head
		tcp_stream_verify_head(p_filter);
	}
	else if (p_filter->recv_index == p_head->length)
	{
		tcp_stream_verify_data(p_filter);
	}
}


void tcp_byte_stream_handler(NETFilter* p_filter, char in_data)
{
	tcp_stream_store_data(p_filter, in_data);
	tcp_check_stream_state(p_filter);
}

void tcp_byte_handle(char in_data)
{
	net_data.reuse_count = 0;
	net_data.reuse_index = _NET_MAX_RECV_SIZE;
	
	tcp_byte_stream_handler(&net_data, in_data);

	if (net_data.reuse_count != 0)
	{
		while (net_data.reuse_index < _NET_MAX_RECV_SIZE)
		{
			in_data = net_data.comm_recv_buf[net_data.reuse_index];
			net_data.reuse_index++;
			tcp_byte_stream_handler(&net_data, in_data);
		}
		net_data.reuse_count = 0;
	}
}


