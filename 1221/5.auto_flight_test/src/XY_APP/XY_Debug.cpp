#include "XY_Debug.h"

static int wireless_fd = -1;
static fd_set wireless_fd_set;

struct debug_info debug_package = {0};

pthread_mutex_t debug_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t debug_cond = PTHREAD_COND_INITIALIZER;


int wireless_open(const char *port_str)
{
	wireless_fd = open(port_str, O_RDWR | O_NOCTTY);  //block mode
	if(wireless_fd < 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	return 0;
}

int wireless_close()
{
	close(wireless_fd);
	wireless_fd = -1;
	return 0;
}

int wireless_flush()
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

int wireless_config(int baudrate, char data_bits, char parity_bits, char stop_bits)
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

static int wireless_start(const char *dev_name, int baud_rate)
{
	const char *ptemp;
	if(dev_name == NULL)
	{
		ptemp = UART1_DEV_NAME;
	}
	else
	{
		ptemp = dev_name;
	}
	if(wireless_open(ptemp) == 0
			&& wireless_config(baud_rate,8,'N',1) == 0)
	{
		FD_ZERO(&wireless_fd_set);
		FD_SET(wireless_fd, &wireless_fd_set);
		return wireless_fd;
	}
	return -1;
	
}


int wireless_send(char *buf,int len)
{
	return write(wireless_fd, buf, len);
}

int wireless_setup(const char *device,int baudrate)
{
	if(wireless_start(device,baudrate) < 0)
	{
		wireless_close();
		return -1;
	}
	if(create_wireless_debug_thread() < 0)
	{
		return -1;
	}

	return 0;
}

void sdk_handle_debug_info(struct debug_info info, char *str, int flag)
{
	switch(flag)
	{
		case 1:
			sprintf(str, "\n[S](%.8lf, %.8lf) [Cur](%.8lf, %.8lf)", 	debug_package.start.longti,
																		debug_package.start.lati,  
																		debug_package.cur.longti,
																		debug_package.cur.lati);
			break;
		case 2:
#if 0
			sprintf(str, "[User Ctrl] roll:%.8f, pitch:%.8f, thr:%.8f, yaw:%.8f\n", debug_package.user_ctrl_data.roll_or_x,
																					debug_package.user_ctrl_data.pitch_or_y, 
																					debug_package.user_ctrl_data.thr_z, 
																					debug_package.user_ctrl_data.yaw);
#endif


			break;
		case 3:
#if 0
			sprintf(str, "[Cur](%.8lf, %.8lf, %.8lf) - [Last Distance] %.8lfm\n",	debug_package.cur.longti,
																					debug_package.cur.lati, 
																					debug_package.cur.alti,
																					debug_package.dist);
#endif
			sprintf(str, "\t[Last Distance] %.8lfm\n",	debug_package.dist);
			break;
			
		default:
			break;
	}

}

int sdk_send_debug_info(char *str)
{
	wireless_send(str, strlen((const char *)str));
	return 0;
}

int sdk_debug_setup(void)
{
	int baudrate = 9600;
	int ret;
	
	ret = wireless_setup(DEBUG_DEV_NAME, baudrate);

	if(ret < 0)
		return ret;

	return 0;
}

int XY_Debug_Setup(void)
{
	return sdk_debug_setup();
}

void XY_Debug_Get_Pos(position *pp, double _longti, double _lati, double _alti)
{
	pp->longti = _longti;
	pp->lati = _lati;
	pp->alti = _alti;
}


void XY_Debug_Get_UserCtrl(attitude *pa, float _roll, float _pitch, float _thr, float _yaw)
{
	pa->roll_or_x= _roll;
	pa->pitch_or_y= _pitch;
	pa->thr_z= _thr;
	pa->yaw = _yaw;
}

void XY_Debug_Get_Last_Dist(double *dst, double _last_distance)
{
	*dst = _last_distance;
}

static void *Wireless_Debug_Thread_Func(void * arg)
{
	char str[200];	
	
	while(1)
	{	
		pthread_mutex_lock(&debug_mutex);
		pthread_cond_wait(&debug_cond , &debug_mutex);	
		pthread_mutex_unlock(&debug_mutex);
		sdk_handle_debug_info(debug_package, str, 1);
		sdk_send_debug_info(str);
		sdk_handle_debug_info(debug_package, str, 3);
		sdk_send_debug_info(str);		
		usleep(1000000);
		
	}
	pthread_exit(NULL);
}


int create_wireless_debug_thread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0, Wireless_Debug_Thread_Func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}


