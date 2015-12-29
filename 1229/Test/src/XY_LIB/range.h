#ifndef __RANGE_H__
#define __RANGE_H__


#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include "wireless_debug.h"


#define KS103_SLAVE_ADDRESS		0xD0
#define KS103_REGISTER2_ADDRESS	0x02
#define KS103_DEFAULT_CMD		0xBC

#define KS103_RETRIES	0x01
#define KS103_TIMEOUT	0x02
#define KS103_SCMD		0x03

#define RANGE_100MM_RUS				0x01
#define RANGE_4700MM_RUS			0x2F
#define RANGE_0_2_5M_RMM_NTC		0xB0
#define RANGE_0_2_5M_RMM_TC			0xB4
#define RANGE_0_2_11M_RMM_NTC		0xB8
#define RANGE_0_2_11M_RMM_TC		0xBC
#define RANGE_12CM_2_11M_RMM_NTC	0xBD
#define RANGE_12CM_2_11M_RMM_TC		0xBF

#define T_9B						0xC9
#define T_10B						0xCA
#define T_11B						0xCB
#define T_12B						0xCC


struct ks103_adapter{
	unsigned char cmd;
	int timeout;
	int retries;
};

int KS103_Send(unsigned char *buf,int len);
int KS103_Setup(const char *device,int baudrate);



#endif
