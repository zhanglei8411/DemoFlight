#ifndef __NETWORK_CHAT_3G_H__
#define __NETWORK_CHAT_3G_H__

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>

#define SERVER_IP		"192.168.3.11"
#define SERVER_PORT		60000

#define FRAME_SIZE		16

#define _NET_MAX_RECV_SIZE          (1024)
#define _NET_SOF                    ((unsigned char)(0xAA))
#define _NET_CRC_HEAD_SIZE          (2)                 // CRC16


typedef struct
{
	unsigned int sof : 8; // 1byte

	unsigned int length : 16;
	unsigned int reversed0 : 7;
	unsigned int is_ack : 1;
	unsigned int head_crc : 16;
	
}NETHeader;


typedef struct
{
	unsigned short reuse_index;
	unsigned short reuse_count;
	unsigned short recv_index;
	unsigned char comm_recv_buf[_NET_MAX_RECV_SIZE];
}NETFilter;


int XY_Chat_Setup(void);
int create_link_in_tcp(void);
static void *chat_recv_thread_func(void * arg);
void tcp_byte_handle(char in_data);

#endif
