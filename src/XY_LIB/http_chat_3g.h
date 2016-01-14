#ifndef __HTTP_CHAT_3G_H__
#define __HTTP_CHAT_3G_H__

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
#include <sys/time.h>
#include <time.h> 
#include <errno.h> 
#include <netdb.h>
#include <stdarg.h>
#include "cJSON.h"




#define SERVER_NAME		"cs.sysmagic.com.cn"
#define SERVER_PORT		(80)

#define HTTP_REQ_SIZE		(512)
#define HTTP_RECV_BUF_SIZE	(1024)
#define HTTP_POST_DATA_SIZE	(1024)


#define DECLARED_POST_MSG	3
#define POST_MSG_1_SIZE		100
#define POST_MSG_2_SIZE		100
#define POST_MSG_3_SIZE		100


struct post_info{
	char msg_0[POST_MSG_1_SIZE];
	char msg_1[POST_MSG_1_SIZE];
	char msg_2[POST_MSG_1_SIZE];
};



int XY_Http_Chat_Setup(void);
int create_link_in_http(void);
static void *reported_data_thread_func(void * arg);
static void *wait_order_thread_func(void * arg);
int XY_Send_Http_Post_Request_Data(int seq, char *fmt, ...);


#endif
