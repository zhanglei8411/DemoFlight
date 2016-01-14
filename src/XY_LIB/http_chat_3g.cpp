/*
 * Description	: Chat by TCP/HTTP
 * Author		: Zibo Zhou
 * Time			: 2016/1/6
 */

#include "http_chat_3g.h"
#include "thread_common_op.h"
#include "wireless_debug.h"


int sock_fd = -1;
char http_post_str[HTTP_POST_DATA_SIZE];
int http_post_head_length = 0;

pthread_mutex_t post_req_flag_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t post_msg_lock = PTHREAD_MUTEX_INITIALIZER;

int post_req_flag = 0;

struct post_info post_package = {0};


void XY_Update_Post_Flag(int _offset)
{
	pthread_mutex_lock(&post_req_flag_lock);
	post_req_flag |= (1<<_offset);
	pthread_mutex_unlock(&post_req_flag_lock);
}

int XY_Get_Post_Flag(void)
{
	int ret;
	pthread_mutex_lock(&post_req_flag_lock);
	ret = post_req_flag;
	pthread_mutex_unlock(&post_req_flag_lock);

	return ret;
}

void XY_Empty_Post_Flag(int _offset)
{
	pthread_mutex_lock(&post_req_flag_lock);
	post_req_flag &= ~(1<<_offset);
	pthread_mutex_unlock(&post_req_flag_lock);
}


int XY_Http_Chat_Setup(void)
{

	if( create_link_in_http() < 0)
	{
		return -1;
	}
	
	if(XY_Create_Thread(wait_order_thread_func, NULL, THREADS_WAIT_ORDER, -1, SCHED_RR, 96) < 0)
	{
		printf("Create Wait Order Thread Error.\n");
		return -1;
	}
	
	if(XY_Create_Thread(reported_data_thread_func, NULL, THREADS_HTTP_POST, -1, SCHED_RR, 94) < 0)
	{
		printf("Create Http Post Thread Error.\n");
		return -1;
	}

	return 0;

}

int create_socket(int *_sock_fd)
{
	if( (*_sock_fd = socket(AF_INET, SOCK_STREAM, 0) ) < 0)
	{
		perror("socket error");
		return -1;
	}
	return 0;
}

int setup_addr(struct sockaddr_in *_addr, unsigned short _port, const char *hostname)
{
	struct hostent *host;
	
	_addr->sin_family = AF_INET;
	_addr->sin_port = htons(_port);
	host = gethostbyname(hostname);
	if(!host)
	{
		printf("get host error.\n");
		return -1;
	}
	_addr->sin_addr = *(struct in_addr *)host->h_addr_list[0];
	bzero(&(_addr->sin_zero), 8);

	return 0;
}

int connect_to_server(int _sock_fd)
{
	struct sockaddr_in server_addr;
	
	if( setup_addr(&server_addr, SERVER_PORT, SERVER_NAME) < 0)
	{
		perror("setup addr");
		return -1;
	}

	printf("* About to connect() to %s port %d.\n", SERVER_NAME, SERVER_PORT);
	if( connect(_sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr) ) == -1)
	{
		perror("connect error");
		return -1;
	}
	printf("*  Trying %s... connected\n", inet_ntoa(server_addr.sin_addr));
	
	return 0;
}

void setup_post_data_head(void)
{
	memset(http_post_str, 0, HTTP_POST_DATA_SIZE);
	strcat(http_post_str, "GET /api/order/get_json_order HTTP/1.1\r\n");
	strcat(http_post_str, "POST api/order/set_express HTTP/1.1\r\n");
	strcat(http_post_str, "User-Agent: curl/7.22.0 (x86_64-pc-linux-gnu) libcurl/7.22.0 OpenSSL/1.0.1 zlib/1.2.3.4 libidn/1.23 librtmp/2.3\r\n");
	strcat(http_post_str, "Host: cs.sysmagic.com.cn\r\n");
	strcat(http_post_str, "\r\n");
	http_post_head_length = strlen((const char *)http_post_str);
}

int create_link_in_http(void)
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

	setup_post_data_head();

	return 0;
}


int send_http_post_data(char *_data)
{
	int index = 0;
	int ret = 0;
	
	index = http_post_head_length;
	memset(http_post_str + index, 0, HTTP_POST_DATA_SIZE - index);

	strcat(http_post_str, _data);
	
	printf("*  Send data in http...\n");
	printf("%s", http_post_str);

	ret = send(sock_fd, (void *)http_post_str, strlen(http_post_str), 0); 
	if (ret < 0)
	{ 
		printf("send error %d, Error message'%s'\n", errno, strerror(errno)); 
		return -1;
	}
	else
	{ 
		printf("send success, total send %d \n", ret); 
	} 

	return 0;
}

int post_msg_size[DECLARED_POST_MSG] = {POST_MSG_1_SIZE, POST_MSG_2_SIZE, POST_MSG_3_SIZE};
int find_available_post_msg(int need_len, int used_msg)
{
	int free_msg = -1;
	int i = 0;
	
	free_msg = (int)(pow(2, DECLARED_POST_MSG)-1) - used_msg;

	if(free_msg == 0)
		return -1;

	while(i < DECLARED_POST_MSG)
	{
		if(free_msg & 0x01)
		{
			if(need_len < post_msg_size[i])
				return i;
		}
		free_msg = free_msg >> 1;	
		i++;
	}

	return -1;
}


int XY_Send_Http_Post_Request_Data(int seq, char *fmt, ...)
{
	va_list args;
	int r;

	if( ((r = XY_Get_Post_Flag()) & (1<<seq)) != 0  || seq == NULL)
	{
		seq = find_available_post_msg( strlen((const char *)fmt), r); 
		if(seq < 0)
			return 1;
	}
	
	va_start(args, fmt);	//使ap指向(fmt的基地址+fmt的长度)内存位置，即第二个参数位置
	pthread_mutex_lock(&post_msg_lock);
	switch(seq)
	{
		case 0:
			memset(post_package.msg_0, 0, strlen((const char *)post_package.msg_0));
			vsprintf(post_package.msg_0, fmt, args);
			break;
		case 1:
			memset(post_package.msg_1, 0, strlen((const char *)post_package.msg_1));
			vsprintf(post_package.msg_1, fmt, args);
			break;
		case 2:
			memset(post_package.msg_2, 0, strlen((const char *)post_package.msg_2));
			vsprintf(post_package.msg_2, fmt, args);
			break;
			break;
	}
	pthread_mutex_unlock(&post_msg_lock);
	va_end(args);
	
	XY_Update_Post_Flag(seq);

	return 0;
}


static void *reported_data_thread_func(void * arg)
{
	int _flag = 0;
	int i = 0;
	

	while(1)
	{	
		usleep(200000);	// 200ms
		if( (_flag = XY_Get_Post_Flag()) == 0)	// no msg should be sent
		{
			continue;
		}
		else
		{
			while(i < (sizeof(_flag)/2) )
			{
				
				if(_flag & 0x01)
				{
					switch(i)
					{
						case 0:
							send_http_post_data(post_package.msg_0);
							break;
						case 1:
							send_http_post_data(post_package.msg_1);
							break;
						case 2:
							send_http_post_data(post_package.msg_2);
							break;
					}
					XY_Empty_Post_Flag(i);
				}
				_flag = _flag >> 1;	
				i++;
			}
			i = 0;
		}
		
	}
	pthread_exit(NULL);
}


static void *wait_order_thread_func(void * arg)
{	
	char http_req[HTTP_REQ_SIZE], recv_buf[HTTP_RECV_BUF_SIZE];
	fd_set http_fds; 
	struct timeval tv;
	int ret, recv_cnt = 0;
	struct timeval tpstart, tpend;
	int timeuse, timesleep;
	int length = 0;
	char json_buf[200] = {0};
	cJSON *json;

	memset(http_req, 0, HTTP_REQ_SIZE);
	strcat(http_req, "GET /api/order/get_json_order HTTP/1.1\r\n");		//request line
	strcat(http_req, "User-Agent: curl/7.22.0 (x86_64-pc-linux-gnu) libcurl/7.22.0 OpenSSL/1.0.1 zlib/1.2.3.4 libidn/1.23 librtmp/2.3\r\n");
	strcat(http_req, "Host: cs.sysmagic.com.cn\r\n");
	strcat(http_req, "Accept: */*\r\n\r\n");
	
	printf("%s", http_req);
	
	while(1)
	{
req_again:
		usleep(1000000);
req_again_timeout:
		ret = send(sock_fd, (void *)http_req, strlen(http_req), 0); 
		if (ret < 0)
		{ 
			printf("send error %d, Error message'%s'\n", errno, strerror(errno)); 
			exit(1); 
		}
		else
		{ 
			printf("send success, total send %d \n", ret); 
		} 
		
		while(1)
		{
			tv.tv_sec = 2; 
			tv.tv_usec = 0; 

			FD_ZERO(&http_fds); 
			FD_SET(sock_fd, &http_fds); 

			gettimeofday(&tpstart, NULL); 
			switch( select(sock_fd + 1, &http_fds, NULL, NULL, &tv) )
			{
				case -1:	// 错误
					//close(sock_fd); 	//是否需要重新连接?
					printf("some thing read error!\n");		
					goto req_again;

				case 0:		// 超时
					printf("request time out!\n");
					goto req_again_timeout;

				default:
					memset(recv_buf, 0, HTTP_RECV_BUF_SIZE); 
					recv_cnt = recv(sock_fd, (void *)recv_buf, HTTP_RECV_BUF_SIZE, 0);		// recv会等到协议把数据接收完毕才会进行copy和返回
					
					printf("recv count = %d\n", recv_cnt); 
					/*
					 * recv return
					 * <0 : copy error
					 * =0 : 网络中断
					 * >0 : 接收到数据大小
					 */ 
					if (recv_cnt == 0)
					{ 
						close(sock_fd); 
						printf("Network errors, stop!\n"); 
						goto _exit;
					} 
					printf("%s\n", recv_buf); 
					break;
			}
			
			// Transfer-Encoding: chunked. chunked数据一定以"0\r\n\r\n"结尾
			if( strstr(recv_buf, "0\r\n\r\n") )
				break;
		}
		gettimeofday(&tpend,NULL); 
		
		if(strstr(recv_buf, "}\r\n") == NULL)
		{
			printf("No valid data, Trying again...\n");
			//timeuse = 1000000 * (tpend.tv_sec - tpstart.tv_sec) + tpend.tv_usec - tpstart.tv_usec;
			//timesleep = 
			goto req_again;
		}
		
		length = strstr(recv_buf, "}") - strstr(recv_buf, "{") + 1;
		//printf("length is %d\n", length);
		memcpy(json_buf, strstr(recv_buf, "{"), length);
		//printf("json_buf is %s\n", json_buf);
		json = cJSON_Parse(json_buf);
		if (!json)
		{
			printf("Error before: [%s]\n", cJSON_GetErrorPtr());
			goto req_again;
		}
		printf("Get: \n> num is %d\n> seq is %d\n> longti is %.8lf\n> lati is %.8lf\n", cJSON_GetObjectItem(json, "num")->valueint, 
																						cJSON_GetObjectItem(json, "seq")->valueint, 
																						cJSON_GetObjectItem(json, "lng")->valuedouble, 
																						cJSON_GetObjectItem(json, "lat")->valuedouble);
		
		XY_Debug_Send_At_Once("\nGet: \n> num is %d\n> seq is %d\n> longti is %.8lf\n> lati is %.8lf\n",cJSON_GetObjectItem(json, "num")->valueint, 
																										cJSON_GetObjectItem(json, "seq")->valueint, 
																										cJSON_GetObjectItem(json, "lng")->valuedouble, 
																										cJSON_GetObjectItem(json, "lat")->valuedouble);
		break;
	}

_exit:
	pthread_exit(NULL);
}



