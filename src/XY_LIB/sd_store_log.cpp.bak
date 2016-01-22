#include "sd_store_log.h"
#include "thread_common_op.h"
#include "image_identify.h"
#include "range.h"
#include "route.h"
#include "control_law.h"
#include "../DJI_LIB/DJI_Pro_App.h"


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
	if(sem_init(&log_start_sem, 0, 0) != 0)
		goto error;

	if(sem_init(&log_ctrl_data_sem, 0, 0) != 0)
		goto error;
	
	sprintf(log_name, "/mnt/sdcard/log/sd-log-%d", get_current_cnt_in_profile() );
	printf("%s\n", log_name);

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
Center_xyz  cur_xyz;
Body_Angle body_angle;
api_common_data_t cur_acc;
api_vel_data_t cur_vo;    
sdk_std_msg_t cur_broadcast_data;

extern float ultra_buf[ULTRA_ANALYSIS_NUM];



void store_depend_stat(int _stat, char *strp)
{
	switch(_stat)
	{
		case 0x01:
			sprintf(strp, "[body_angle- roll_deg: %.4f,pitch_deg : %.4f, yaw_deg %.4f];[cur_acc- agx: %.4f, agy: %.4f, agz%.4f];[cur_vo- vgx: %.4f, vgy: %.4f, vgz%.4f];[cur_w- wx: %.4f, wy: %.4f, wz%.4f];[cur_mag- mx: %d, my: %d, mz: %d];",      
                                                                                                               body_angle.roll_deg, body_angle.pitch_deg, body_angle.yaw_deg,
                                                                                                               cur_acc.x, cur_acc.y, cur_acc.z,
                                                                                                               cur_vo.x, cur_vo.y, cur_vo.z,
                                                                                                               cur_broadcast_data.w.x, cur_broadcast_data.w.y, cur_broadcast_data.w.z,
                                                                                                               cur_broadcast_data.mag.x, cur_broadcast_data.mag.y, cur_broadcast_data.mag.z);
			break;
		case 0x02:
			sprintf(strp, "[pos - longti: %.10lf, lati: %.10lf, alti: %.8f, height: %.8f, health: %x];", 	_log_pos.longti,
																								_log_pos.lati,
																								_log_pos.alti,
																								_log_pos.height,
																								_log_pos.health_flag);
			break;
			
		case 0x04:
			sprintf(strp, "[cur_xyz.x %.3lf, cur_xyz.y %.3lf, cur_xyz.z %.3lf];", 	cur_xyz.x,
																					cur_xyz.y,
																					cur_xyz.z);
			break;
			
		case 0x08:
			break;
		case 0x10:
			sprintf(strp, "[ctrl data - pitch: %.f, roll: %f, yaw: %f, thr: %f] ", 	_log_user_ctrl_data.pitch_or_y,
																					_log_user_ctrl_data.roll_or_x,
																					_log_user_ctrl_data.yaw,
																					_log_user_ctrl_data.thr_z);
			break;
			
        case 0x20:
			sprintf(strp, "[ultra_data: %.4f];", _log_ultra_data);
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
      
	api_pos_data_t g_origin_pos;
	//原点在羽毛球场起飞点
	g_origin_pos.longti = ORIGIN_IN_HENGSHENG_LONGTI;
	g_origin_pos.lati = ORIGIN_IN_HENGSHENG_LATI;
	g_origin_pos.alti = ORIGIN_IN_HENGSHENG_ALTI;
	XYZ g_origin_XYZ, cur_XYZ;  
	api_quaternion_data_t cur_quaternion;
	struct timeval    tv;  
	struct tm         *tmlocal; 
	double dji_time=0;
	char _wbuf[500] = {0};

	if(log_fd == -1)
	{
		printf("Bad log fd.\n");
		pthread_exit(NULL);
	}
	
	while(1)
	{	
		if(if_log_on() )
		{
#if 0
			/* sd code should use OFFSET_GET_ID_B */
			Offset offset;
			if( XY_Get_Offset_Data(&offset, OFFSET_GET_ID_B) == 0)
			{
				/* ... */
			}
#endif
			
		
			DJI_Pro_Get_Quaternion(&cur_quaternion);
			QUA2ANGLE(cur_quaternion,&body_angle) ;                                                        
			DJI_Pro_Get_GroundAcc(&cur_acc);  
			DJI_Pro_Get_GroundVo(&cur_vo);
			DJI_Pro_Get_Broadcast_Data(&cur_broadcast_data);
			stat |= 0x01;
                              
			stat |= 0x08;
			DJI_Pro_Get_Pos(&_log_pos);
			geo2XYZ(_log_pos, &cur_XYZ);
			XYZ2xyz(g_origin_pos, cur_XYZ, &cur_xyz);
			stat |= (0x02 | 0x04);
			
			if( XY_Get_Attitude_Data(&_log_user_ctrl_data) == 0 )
			{
				stat |= 0x10;
			}
			
			if(XY_Get_Ultra_Data(&_log_ultra_data, ULTRA_GET_ID_B) == 0)
		   	{
			   	stat |= 0x20;
		   	}
                              
			if( gettimeofday(&tv, NULL) != 0)
			{
				printf("ERROR get timestamp for log.\n");
				
			}
			tmlocal = localtime(&tv.tv_sec); 
        	dji_time=((float)cur_broadcast_data.time_stamp/600);
			
			sprintf(_wbuf, "[%d-%d %d:%d:%d:%6ld]; [dji time %6lf];",	1 + tmlocal->tm_mon,
														tmlocal->tm_mday,
														tmlocal->tm_hour,
														tmlocal->tm_min,
														tmlocal->tm_sec,
														tv.tv_usec,
														dji_time); 
			
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
			check_stat_store(stat, 0x10,_wbuf);
			check_stat_store(stat, 0x20,_wbuf);
		  
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

