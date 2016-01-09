#include "sd_store_log.h"
#include "thread_common_op.h"
#include "image_identify.h"
#include "range.h"


#define CRLF "\n"


sem_t log_ctrl_data_sem;
sem_t log_start_sem;
pthread_mutex_t log_ctrl_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t log_on_off_lock = PTHREAD_MUTEX_INITIALIZER;
int log_on_flag = 0;


int log_fd = -1;
char log_name[50];

int init_log(int *fd)
{
	struct timeval    tv;  
    struct tm         *tmlocal; 

	if(sem_init(&log_start_sem, 0, 0) != 0)
		goto error;

	if(sem_init(&log_ctrl_data_sem, 0, 0) != 0)
		goto error;
	
	if( gettimeofday(&tv, NULL) != 0)
	{
		printf("ERROR init log when get time.\n");
		goto error;
	}
	tmlocal = localtime(&tv.tv_sec); 
	sprintf(log_name, "/mnt/sdcard/log/%d_%d_%d_%d_%d_%d.config", 	1900 + tmlocal->tm_year, 
																	1 + tmlocal->tm_mon,
																	tmlocal->tm_mday,
																	tmlocal->tm_hour,
																	tmlocal->tm_min,
																	tmlocal->tm_sec); 
	
	*fd = open(log_name, O_RDWR | O_CREAT | O_APPEND, 0644);
	if(*fd == -1)
	{
		printf("Open or create log file failed, maybe no sd card.\n");
		goto release;
	}
		
	return 0;
	
release:
	memset(log_name, 0, strlen((const char *)log_name));

error:
	return 1;
}


int XY_SD_Log_Setup(void)
{
	if(init_log(&log_fd) != 0)
	{
		log_fd = -1;
		printf("log_fd is -1.\n");
		return -1;
	}

	if(XY_Create_Thread(store_to_log_thread_func, NULL, THREADS_LOG, -1, SCHED_RR, 89) < 0)
	{
		printf("Create Led Show Status Thread Error.\n");
		return -1;
	}

	return 0;
}

float _log_ultra_data;
Offset _log_offset;
api_pos_data_t _log_pos;
attitude_data_t _log_user_ctrl_data;


void store_depend_stat(int _stat, char *strp)
{
	switch(_stat)
	{
		case 0x01:
			sprintf(strp, "[ultra_data: %.4f] ", _log_ultra_data);
			break;
		case 0x02:
			sprintf(strp, "[offset - x: %.6f, y: %.6f, z: %.6f] ", _log_offset.x, _log_offset.y, _log_offset.z);
			break;
		case 0x04:
			sprintf(strp, "[pos - longti: %.10lf, lati: %.10lf, alti: %.8f, height: %.8f] ", 	_log_pos.longti,
																								_log_pos.lati,
																								_log_pos.alti,
																								_log_pos.height);
			break;
		case 0x08:
			sprintf(strp, "[ctrl data - pitch: %.f, roll: %f, yaw: %f, thr: %f] ", 	_log_user_ctrl_data.pitch_or_y,
																					_log_user_ctrl_data.roll_or_x,
																					_log_user_ctrl_data.yaw,
																					_log_user_ctrl_data.thr_z);
			break;
	}
	int ret = write(log_fd, strp, strlen((const char *)strp));
	memset(strp, 0, strlen((const char *)strp));
	if(ret == -1)
	{
		printf("Bad write.\n");
	}
}

void check_stat_store(int _stat, int _mask, char *strp)
{
	if(_stat & _mask)
		store_depend_stat(_mask, strp);
}

void add_crlf(void)
{
	write(log_fd, CRLF, 1);
}

attitude_data_t _attitude_data;
void set_attitude_data(attitude_data_t _user_ctrl_data)
{
	pthread_mutex_lock(&log_ctrl_lock);
	_attitude_data.pitch_or_y 	= _user_ctrl_data.pitch_or_y;
	_attitude_data.roll_or_x 	= _user_ctrl_data.roll_or_x;
	_attitude_data.yaw 			= _user_ctrl_data.yaw;
	_attitude_data.thr_z 		= _user_ctrl_data.thr_z;
	pthread_mutex_unlock(&log_ctrl_lock);
	sem_post(&log_ctrl_data_sem);
}

attitude_data_t get_attitude_data(void)
{
	attitude_data_t ret;
	pthread_mutex_lock(&log_ctrl_lock);
	ret = _attitude_data;
	pthread_mutex_unlock(&log_ctrl_lock);

	return ret;
}

int check_attitude_data_if_available(void)
{
	int i = 0;
	while(sem_trywait(&log_ctrl_data_sem) == 0)
	{
		i++;
	}
	if(i != 0)
		return 0;							//available
	else
		return -1;
}


int XY_Get_Attitude_Data(attitude_data_t *_data)
{
	if(check_attitude_data_if_available() == -1)
	{
		return -1;
	}

	*_data = get_attitude_data();
	return 0;
}

void set_log_on_flag(int _val)
{
	_val = _val > 1 ? 1 : _val;
	pthread_mutex_lock(&log_on_off_lock);
	log_on_flag = _val;
	pthread_mutex_unlock(&log_on_off_lock);
}

void clear_log_on_flag(void)
{
	set_log_on_flag(0);
}

int get_log_on_flag(void)
{
	int ret;
	pthread_mutex_lock(&log_on_off_lock);
	ret = log_on_flag;
	pthread_mutex_unlock(&log_on_off_lock);

	return ret;
}

int if_log_on(void)
{
	return get_log_on_flag();
}

void wait_log_on(void)
{
	sem_wait(&log_start_sem);
}


int XY_Start_Store_Log(void)
{
	set_log_on_flag(1);
	//sem_post(&log_start_sem);

	return 0;
}

int XY_Stop_Store_Log(void)
{
	set_log_on_flag(0);

	return 0;
}



/* [timestamp]  */
static void *store_to_log_thread_func(void * arg)
{	

	int stat = 0;

	struct timeval    tv;  
    struct tm         *tmlocal; 

	char _wbuf[100] = {0};

	if(log_fd == -1)
	{
		printf("Bad log fd.\n");
		pthread_exit(NULL);
	}
	
	while(1)
	{	
		if(if_log_on() )
		{
		
			if(XY_Get_Ultra_Data(&_log_ultra_data) == 0)
		   	{
			   	stat |= 0x01;
		   	}
		
			if( XY_Get_Offset_Data(&_log_offset) == 0)
			{
				stat |= 0x02;
			}
			
			DJI_Pro_Get_Pos(&_log_pos);
			stat |= 0x04;
			
			if( XY_Get_Attitude_Data(&_log_user_ctrl_data) == 0 )
			{
				stat |= 0x08;
			}

			if( gettimeofday(&tv, NULL) != 0)
			{
				printf("ERROR get timestamp for log.\n");
				
			}
			tmlocal = localtime(&tv.tv_sec); 

			
			sprintf(_wbuf, "[%d-%d %d:%d:%d:%6ld] ",	1 + tmlocal->tm_mon,
														tmlocal->tm_mday,
														tmlocal->tm_hour,
														tmlocal->tm_min,
														tmlocal->tm_sec,
														tv.tv_usec); 
			
			int ret = write(log_fd, _wbuf, strlen((const char *)_wbuf));
			memset(_wbuf, 0, strlen((const char *)_wbuf));
			if(ret == -1)
			{
				perror("log write");
			}
			check_stat_store(stat, 0x01, _wbuf);
			check_stat_store(stat, 0x02, _wbuf);
			check_stat_store(stat, 0x04, _wbuf);
			check_stat_store(stat, 0x08, _wbuf);

			add_crlf();
			
			stat = 0;
			
			usleep(100000);	//100ms
		}
		else
		{
			printf("log fd will be closed.\n");
			close(log_fd);
			break;
			//wait_log_on();
		}
	}
	pthread_exit(NULL);
}

