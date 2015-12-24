#include "XY_Route.h"

static int wireless_fd = -1;
static fd_set wireless_fd_set;

Wl_Filter wl = {0};
int route_task_arrived = 0;

Link_Leg_Node *head;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;


extern int express_task_running;
extern int status_ctrl_lock;
extern Link_Leg_Node *leg_head;
Link_Leg_Node *cur_legn = NULL;


int WirelessOpen(const char *port_str)
{
	wireless_fd = open(port_str, O_RDWR | O_NOCTTY);  //block mode
	if(wireless_fd < 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	return 0;
}

int WirelessClose()
{
	close(wireless_fd);
	wireless_fd = -1;
	return 0;
}

int WirelessFlush()
{
	if(wireless_fd < 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
	}
	else
	{
		tcflush(wireless_fd, TCIFLUSH);	//清除正收到的数据
	}
	return 0;
}

int WirelessConfig(int baudrate, char data_bits, char parity_bits, char stop_bits)
{
	int st_baud[]=
	{
		B4800,
		B9600,
		B19200,
		B38400,
		B57600,
		B115200,
		B230400
	};
	int std_rate[]=
	{
		4800,
		9600,
		19200,
		38400,
		57600,
		115200,
		230400,
		1000000,
		1152000,
		3000000,
	};

	int i,j;
	struct termios newtio, oldtio;
	/* save current port parameter */
	if (tcgetattr(wireless_fd, &oldtio) != 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	bzero(&newtio, sizeof(newtio));

	/* config the size of char */
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	/* config data bit */
	switch (data_bits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
	/* config the parity bit */
	switch (parity_bits)
	{
		/* odd */
	case 'O':
	case 'o':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		break;
		/* even */
	case 'E':
	case 'e':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
		/* none */
	case 'N':
	case 'n':
		newtio.c_cflag &= ~PARENB;
		break;
	}
	/* config baudrate */
	j = sizeof(std_rate)/4;
	for(i = 0;i < j;i ++)
	{
		if(std_rate[i] == baudrate)
		{
			/* set standard baudrate */
			cfsetispeed(&newtio, st_baud[i]);
			cfsetospeed(&newtio, st_baud[i]);
			break;
		}
	}
	/* config stop bit */
    if( stop_bits == 1 )
       newtio.c_cflag &=  ~CSTOPB;
    else if ( stop_bits == 2 )
       newtio.c_cflag |=  CSTOPB;

    /* config waiting time & min number of char */
    newtio.c_cc[VTIME]  = 1;
    newtio.c_cc[VMIN] = 1;
	/* using the raw data mode */
	newtio.c_lflag	&= ~(ICANON | ECHO | ECHOE | ISIG);
	newtio.c_oflag	&= ~OPOST;

	/* flush the hardware fifo */
	tcflush(wireless_fd, TCIFLUSH);

	/* activite the configuration */
	if((tcsetattr(wireless_fd, TCSANOW,&newtio))!=0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	return 0;
}




static int WirelessStart(const char *dev_name, int baud_rate)
{
	const char *ptemp;
	if(dev_name == NULL)
	{
		ptemp = "/dev/ttyUSB1";
	}
	else
	{
		ptemp = dev_name;
	}
	if(WirelessOpen(ptemp) == 0
			&& WirelessConfig(baud_rate,8,'N',1) == 0)
	{
		FD_ZERO(&wireless_fd_set);
		FD_SET(wireless_fd, &wireless_fd_set);
		return wireless_fd;
	}
	return -1;
	
}


static void *Wireless_Recv_Thread_Func(void * arg)
{
	int ret;
	unsigned int depth,len;
	unsigned char buf[64];
	int i;
	
	while(1)
	{
		//阻塞，直到有文件描述符就绪
		ret = select(FD_SETSIZE, &wireless_fd_set, (fd_set *)0, (fd_set *)0, (struct timeval *)0);
		if (ret < 1)
		{
			printf("%s,%d,ERROR\n", __func__, __LINE__);
			FD_ZERO(&wireless_fd_set);
			FD_SET(wireless_fd, &wireless_fd_set);
			continue;
		}
		//得到当前缓冲区等待被读取的字节数
		ioctl(wireless_fd, FIONREAD, &depth);
		if(depth > 0)
		{
			len = depth > sizeof(buf) ? sizeof(buf) : depth;
			ret = read(wireless_fd, buf,len);
			for(i = 0; i < ret; i ++)
			{	
				byte_stream_handler(&wl, buf[i]);
			}
		}
		
	}
	pthread_exit(NULL);
}

int WirelessStartThread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0, Wireless_Recv_Thread_Func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}


void stream_store_wireless_data(Wl_Filter* p_filter, unsigned char in_data)
{
	//保存数据
	if (p_filter->recv_index < _WL_MAX_RECV_SIZE)
	{
		p_filter->comm_recv_buf[p_filter->recv_index] = in_data;
		p_filter->recv_index++;
	}
	else
	{
		//清零
		memset(p_filter->comm_recv_buf, 0, p_filter->recv_index);
		p_filter->recv_index = 0;
	}
}


void stream_shift_data(Wl_Filter* p_filter)
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

void parse_data_into_leg(unsigned char *pdata, Leg *leg)
{
	memcpy((unsigned char *)&leg->leg_seq, pdata, 1);
	memcpy((unsigned char *)&leg->longti_e, pdata+1, 8);
	memcpy((unsigned char *)&leg->lati_e, pdata+9, 8);
}

void stream_verify_data(Wl_Filter* p_filter)
{
	Wl_Header* p_head = (Wl_Header*)(p_filter->comm_recv_buf);
	unsigned char *pdata = (unsigned char *)&p_head->leg_num;
	Leg leg;
	int ret;

	if(express_task_running == 0)
	{
		ret = Xy_Setup_List_Head();
		if(ret)
		{
			printf("Set List Head ERROR\n");
			return;
		}
	}
	else
	{
		printf("Task of Express is running, reject the new task!\n");
		return;
	}


	pdata += 1;
	while(1)
	{
		//1.解析数据到leg
		parse_data_into_leg(pdata, &leg);
		pdata += 17;
		//2.记录起始经纬度和当前经纬度到leg
		/* 空 */
		//3.插入数据到链表
		Add_Leg_Node(head, leg);
		if(leg.leg_seq == p_head->leg_num)
		{
			break;
		}
	}	
}


void stream_verify_head(Wl_Filter* p_filter)
{
	Wl_Header* p_head = (Wl_Header*)(p_filter->comm_recv_buf);

	if (p_head->sof == 0xBB)
	{
		
	}
	else
	{
		//后面的数据前移，顶掉第一个字节
		stream_shift_data(p_filter);
	}
}

//检查是否为航路任务信息
void check_stream_state(Wl_Filter* p_filter)
{
	Wl_Header *p_head = (Wl_Header*)(p_filter->comm_recv_buf);
	if(p_filter->recv_index < sizeof(Wl_Header))
	{

	}
	else if(p_filter->recv_index == sizeof(Wl_Header))
	{
		stream_verify_head(p_filter);
	}
	else if (p_filter->recv_index == p_head->length)
	{
		stream_verify_data(p_filter);
		route_task_arrived = 1;		//任务已经到达并且解析和插入完毕
	}
	
}


//数据流的字节处理
void byte_stream_handler(Wl_Filter* p_filter, unsigned char in_data)
{
	stream_store_wireless_data(p_filter, in_data);
	check_stream_state(p_filter);
}

int Wireless_Setup(const char *device,int baudrate)
{
	if(WirelessStart(device,baudrate) < 0)
	{
		WirelessClose();
		return -1;
	}

	if(WirelessStartThread() < 0)
	{
		return -1;
	}

	return 0;
}

int Xy_Waiting_Task_Arrived(void)
{
	int ret;
	ret = Wireless_Setup("/dev/ttyUSB1", 9600);
	if(ret < 0)
		return ret;
		
	while(route_task_arrived != 1);
	return route_task_arrived;
}

int Xy_Setup_List_Head(void)
{ 
	head = Create_Leg_Head(); 
	if(head == NULL)
		return 1;
	return 0;
}

static void *Xy_Aircraft_UpDown_Thread_Func(void * arg)
{
	api_vel_data_t cur_vel;
	api_pos_data_t cur_pos;
	attitude_data_t user_ctrl_data;
	int flag = 0;
	
	while(1)
	{
		/* 1. get cur height from M100 */
		DJI_Pro_Get_Pos(&cur_pos);
		/* 2. get cur vel from M100 */
		DJI_Pro_Get_GroundVo(&cur_vel);


		if( (int)(*(int *)arg) == 1 )
		{
			/* 3. cal attitude data by vel and height  */
			//Cal_Attitude_Ctrl_Data(cur_vel, cur_pos, SAFETY_HEIGHT, &user_ctrl_data, &flag);
			/* 
			 * 3.1 check if reach the SAFETY_HEIGHT
			 * 3.2 cal ctrl data according to (SAFETY_HEIGHT - cur_pos.height)
			 */
			Cal_Attitude_Ctrl_Data_UpDown(cur_vel, cur_pos, SAFETY_HEIGHT, &user_ctrl_data, &flag);
		}
		else if( (int)(*(int *)arg) == 0 )
		{
			/* 3. cal attitude data by vel and height  */
			//Cal_Attitude_Ctrl_Data(cur_vel, cur_pos, LOW_HEIGHT, &user_ctrl_data, &flag);
			/* 
			 * 3.1 LOW_HEIGHT和SAFETY_HEIGHT是目标高度
			 * 3.2 (SAFETY_HEIGHT - cur_pos.height) 或 (cur_pos.height - LOW_HEIGHT) 是否小于某个可视为到达的距离值
			 * 是: flag置1，退出
			 * 否: 根据当前剩余距离计算user_ctrl_data
			 */
			 Cal_Attitude_Ctrl_Data_UpDown(cur_vel, cur_pos, LOW_HEIGHT, &user_ctrl_data, &flag);
		}
		

		if(flag == 1)
		{
			goto exit;
		}
		/* 4. use attitude ctrl data to change the action of drone */
		DJI_Pro_Attitude_Control(&user_ctrl_data);

		/* 5. sleep 20ms */
		usleep(20000);
		
	}

exit:
	pthread_mutex_lock(&mutex);
	pthread_cond_signal(&cond);	//释放条件变量
	pthread_mutex_unlock(&mutex);
	
	pthread_exit(NULL);

}

static void *Xy_Aircraft_P2P_Thread_Func(void * arg)
{
	api_vel_data_t cur_vel;
	api_pos_data_t cur_pos;
	attitude_data_t user_ctrl_data;
	int flag = 0;
	
	while(1)
	{
		/* 1. get cur height from M100 */
		DJI_Pro_Get_Pos(&cur_pos);
		/* 2. get cur vel from M100 */
		DJI_Pro_Get_GroundVo(&cur_vel);
		/* 3. set cur pos in cur_legn */
		Set_Leg_Cur_Pos(&cur_legn->leg, cur_pos.longti, cur_pos.lati, cur_pos.alti);
		
		/* 4. cal attitude data by cur vel, cur pos and goal pos  */
		//Cal_Attitude_Ctrl_Data(cur_vel, cur_pos, cur_legn, &user_ctrl_data, &flag);
		/* 
		 * 4.1 根据cur_pos中的当前经纬度高度和cur_legn中的目标点经纬度高度，计算三维空间上的距离
		 * 4.2 剩余距离是否小于某个允许值
		 * 是:flag置一,退出
		 * 否:根据剩余距离和当前速度计算user_ctrl_data
		 */
		Cal_Attitude_Ctrl_Data_P2P(cur_vel, cur_pos, SAFETY_HEIGHT, cur_legn, &user_ctrl_data, &flag);

		if(flag == 1)
		{
			goto exit;
		}
		/* 5. use attitude ctrl data to change the action of drone */
		DJI_Pro_Attitude_Control(&user_ctrl_data);

		/* 6. sleep 20ms */
		usleep(20000);
		
	}

exit:
	pthread_mutex_lock(&mutex);
	pthread_cond_signal(&cond);
	pthread_mutex_unlock(&mutex);
	
	pthread_exit(NULL);

}

static void *Express_Task_Thread_Func(void * arg)
{
	int ret;
	pthread_t A_ARR;
	int Up = 1;

	if(leg_head->next == NULL)
	{
		goto error;
	}
	cur_legn = leg_head->next;

#if 0
	printf("(End)longti:%lf, lati:%lf, alti:%lf; (Start)longti:%lf, lati:%lf, alti:%lf.\n", cur_legn->leg.longti_e,
																							cur_legn->leg.lati_e,
																							cur_legn->leg.alti_e,
																							cur_legn->leg.longti_s, 
																							cur_legn->leg.lati_s,
																							cur_legn->leg.alti_s);
#endif

	while(1)
	{
		/* --------------------- START ------------------------- */
		/* 起飞完成后高度 - 1.27m */
		/* 1.1 aircraft take off */
		ret = DJI_Pro_Status_Ctrl(4,0);
		/* 1.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);	//一旦进入wait, 会释放mutex的lock, 等收到signal信号就会自动重新获取mutex的lock	
		pthread_mutex_unlock(&mutex);
		/* 1.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */		






		
		/* --------------------- START ------------------------- */
		/* 2.1 aircraft up  */
		Up = 1;
		if( pthread_create(&A_ARR, 0, Xy_Aircraft_UpDown_Thread_Func, &Up) != 0  )
		{
			goto error;
		}
		/* 2.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 2.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */






		

		/* --------------------- START ------------------------- */
		/* 3.1 point to piont leg fly */				
		if( pthread_create(&A_ARR, 0, Xy_Aircraft_P2P_Thread_Func, NULL) != 0  )
		{
			goto error;
		}
		/* 3.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 3.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */







		/* --------------------- START ------------------------- */
		
		/* 4. find mark */
		//												//undefined this pthread

		/* ---------------------- END -------------------------- */






		
		/* --------------------- START ------------------------- */
		/* 5.1 aircraft down */
		Up = 0;
		if( pthread_create(&A_ARR, 0, Xy_Aircraft_UpDown_Thread_Func, &Up) != 0  )
		{
			goto error;
		}
		/* 5.2 wait */
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);
		/* 5.3 check action if success */
		//NULL
		/* ---------------------- END -------------------------- */





		
		/* --------------------- START ------------------------- */
		
		/* 6. aircraft down use ultrasonic / aircraft land */
		ret = DJI_Pro_Status_Ctrl(6,0);
		pthread_mutex_lock(&mutex);
		pthread_cond_wait(&cond , &mutex);
		pthread_mutex_unlock(&mutex);

		/* ---------------------- END -------------------------- */






		if(cur_legn->next != NULL)
		{
			cur_legn = cur_legn->next;
		}
		else
		{
			goto exit;
		}
	}

error:
	//回收资源

exit:
	pthread_exit(NULL);
}


int Xy_Start_Express_Task_Thread(pthread_t *_thread)
{
	int ret;
	
	ret = pthread_create(_thread, 0, Express_Task_Thread_Func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}

FILE* Xy_Open_Data_File(void)
{
	FILE *in;
	in = fopen("data.in", "a+");
	if(in == NULL)
	{
		fprintf(stderr, "Can't open inputfile.\n");
		return 0;
	}
	else 
		return in;
}




