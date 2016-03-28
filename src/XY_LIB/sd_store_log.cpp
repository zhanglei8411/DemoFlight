#include "sd_store_log.h"
#include "thread_common_op.h"
#include "range.h"
#include "route.h"
#include "control_law.h"
#include "../DJI_LIB/DJI_Pro_App.h"


#define CRLF "\n"

sem_t log_ctrl_data_sem;
sem_t log_start_sem;
pthread_mutex_t log_ctrl_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t log_no_gps_z_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t log_on_off_lock = PTHREAD_MUTEX_INITIALIZER;
int log_on_flag = 0;


int log_fd = -1;
char log_name[50];

static void *store_to_log_thread_func(void * arg);

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
		printf("Create Sd store Status Thread Error.\n");
		return -1;
	}

	return 0;
}

float _log_ultra_data;
float _calc_ultra_data;
Offset _log_offset;
Offset _log_offset_adjust;
api_pos_data_t _log_pos;
attitude_data_t _log_user_ctrl_data;
Center_xyz  cur_xyz;
Body_Angle body_angle;
api_common_data_t cur_acc;
api_vel_data_t cur_vo;    
sdk_std_msg_t cur_broadcast_data;
unsigned char cur_battery_remaining;
api_ctrl_info_data_t cur_ctrl_info;


float _log_no_gps_z;

extern float ultra_buf[ULTRA_ANALYSIS_NUM];



void store_depend_stat(int _stat, char *strp)
{
	switch(_stat)
	{
		case 0x01:
			sprintf(strp, "%.4f,%.4f,%.4f;%.4f,%.4f,%.4f;%.4f,%.4f,%.4f,%d;%.4f,%.4f,%.4f;%d,%d,%d;%d;%d;", body_angle.roll_deg, body_angle.pitch_deg, body_angle.yaw_deg,
                                                                                                   cur_acc.x, cur_acc.y, cur_acc.z,
                                                                                                   cur_vo.x, cur_vo.y, cur_vo.z,cur_vo.health_flag,
                                                                                                   cur_broadcast_data.w.x, cur_broadcast_data.w.y, cur_broadcast_data.w.z,
                                                                                                   cur_broadcast_data.mag.x, cur_broadcast_data.mag.y, cur_broadcast_data.mag.z,
                                                                                                   cur_battery_remaining,
                                                                                                   cur_ctrl_info.cur_ctrl_dev_in_navi_mode);
			break;
		case 0x02:
			sprintf(strp, "%.10lf;%.10lf;%.8f;%.8f;%x;", 	_log_pos.longti,
															_log_pos.lati,
															_log_pos.alti,
															_log_pos.height,
															_log_pos.health_flag);
			break;
			
		case 0x04:
			sprintf(strp, "%.3lf,%.3lf,%.3lf;", 	cur_xyz.x,
													cur_xyz.y,
													cur_xyz.z);
			break;
			
		case 0x08:
			sprintf(strp, "%f;", 	_log_no_gps_z);
			break;
			
		case 0x10:
			sprintf(strp, "%f,%f,%f,%f;", 	_log_user_ctrl_data.pitch_or_y,
											_log_user_ctrl_data.roll_or_x,
											_log_user_ctrl_data.yaw,
											_log_user_ctrl_data.thr_z);
			break;
			
        case 0x20:
			sprintf(strp, "%.4f,%.4f;", _log_ultra_data,_calc_ultra_data);
			break;
			
		case 0x40:
			sprintf(strp, "%.4f;%.4f;%.4f;", 	_log_offset.x,
												_log_offset.y,
												_log_offset.z);
			break;
			
		case 0x80:
			sprintf(strp, "%.4f;%.4f;%.4f;", 	_log_offset_adjust.x,
												_log_offset_adjust.y,
												_log_offset_adjust.z);
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
	else	//no valid data
	{
		int ret = write(log_fd, ";", 1);
		if(ret == -1)
		{
			printf("Bad write.\n");      
		}
	}
}

void add_crlf(void)
{
	int ret = 0;
	ret = write(log_fd, CRLF, 1);
	if(ret == -1)
	{
		perror("write");
	}
}

float temp_no_gps_z = 0;
void set_no_gps_z_data(float _no_gps_z)
{
	pthread_mutex_lock(&log_no_gps_z_lock);
	temp_no_gps_z = _no_gps_z;
	pthread_mutex_unlock(&log_no_gps_z_lock);
}

float get_no_gps_z_data(void)
{
	float ret;
	pthread_mutex_lock(&log_no_gps_z_lock);
	ret = temp_no_gps_z;
	temp_no_gps_z = 0;
	pthread_mutex_unlock(&log_no_gps_z_lock);

	return ret;
}


attitude_data_t temp_ctrl_data;
void set_ctrl_data(attitude_data_t _ctrl_data)
{
	pthread_mutex_lock(&log_ctrl_lock);
	temp_ctrl_data.pitch_or_y 	= _ctrl_data.pitch_or_y;
	temp_ctrl_data.roll_or_x 	= _ctrl_data.roll_or_x;
	temp_ctrl_data.yaw 			= _ctrl_data.yaw;
	temp_ctrl_data.thr_z 		= _ctrl_data.thr_z;
	pthread_mutex_unlock(&log_ctrl_lock);
	sem_post(&log_ctrl_data_sem);
}

attitude_data_t get_ctrl_data(void)
{
	attitude_data_t ret;
	pthread_mutex_lock(&log_ctrl_lock);
	ret = temp_ctrl_data;
	pthread_mutex_unlock(&log_ctrl_lock);

	return ret;
}

int check_ctrl_data_if_available(void)
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


int XY_Get_Ctrl_Data(attitude_data_t *_data)
{
	if(check_ctrl_data_if_available() == -1)
	{
		return -1;
	}

	*_data = get_ctrl_data();
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


Offset offset_temp;
pthread_mutex_t offset_adjust_lock = PTHREAD_MUTEX_INITIALIZER;
void set_log_offset_adjust(Offset _src_offset)
{
	pthread_mutex_lock(&offset_adjust_lock);
	offset_temp = _src_offset;
	pthread_mutex_unlock(&offset_adjust_lock);
}

void get_log_offset_adjust(Offset *_offset)
{
	pthread_mutex_lock(&offset_adjust_lock);
	*_offset = offset_temp;
	pthread_mutex_unlock(&offset_adjust_lock);
}
	


/* [timestamp]  */
static void *store_to_log_thread_func(void * arg)
{	

	int stat = 0;
      
	api_pos_data_t g_origin_pos;
	//ԭ������ë����ɵ�
	g_origin_pos.longti = ORIGIN_IN_HENGSHENG_LONGTI;
	g_origin_pos.lati = ORIGIN_IN_HENGSHENG_LATI;
	g_origin_pos.alti = ORIGIN_IN_HENGSHENG_ALTI;
	XYZ cur_XYZ;  
	api_quaternion_data_t cur_quaternion;
	struct timeval    tv;  
	struct tm         *tmlocal; 
	double dji_time=0;
	char _wbuf[1024] = {0};
    Queue* pq=create(4);
       
	if(log_fd == -1)
	{
		printf("Bad log fd.\n");
		pthread_exit(NULL);
	}

	char header[300];
	sprintf(header,
			"linux time;dji time;"
			"body_roll,body_pitch,body_yaw;"
			"agx,agy,agz;"
			"vgx,vgy,vgz,vghealth;"
			"wx,wy,wz;"
			"mx,my,mz;"
			"battery;"
			"status;"
			"longti;lati;alti;height;health;"
			"cur_xyz;"
			"no_gps_z;"
			"pitch,roll,yaw,thr;"
			"ultra,ultra_calc;"
			"of_x;of_y;of_z;"
			"of_ad_x;of_ad_y;of_ad_z;\n");
	
	//printf("%s\n", header);
	
	int ret = write(log_fd, header, strlen((const char *) header));
	if(ret == -1)
	{
		printf("Header bad write.\n");      
	}
	
	while(1)
	{	
		if(if_log_on() )
		{		
			DJI_Pro_Get_Quaternion(&cur_quaternion);
			QUA2ANGLE(cur_quaternion,&body_angle) ;                                                        
			DJI_Pro_Get_GroundAcc(&cur_acc);  
			DJI_Pro_Get_GroundVo(&cur_vo);
			DJI_Pro_Get_Broadcast_Data(&cur_broadcast_data);
			DJI_Pro_Get_Bat_Capacity(&cur_battery_remaining);
			DJI_Pro_Get_CtrlInfo(&cur_ctrl_info);
			stat |= 0x01;
                              
			DJI_Pro_Get_Pos(&_log_pos);
			geo2XYZ(_log_pos, &cur_XYZ);
			XYZ2xyz(g_origin_pos, cur_XYZ, &cur_xyz);
			stat |= (0x02 | 0x04);

			_log_no_gps_z = get_no_gps_z_data();
			stat |= 0x08;
			
			if( XY_Get_Ctrl_Data(&_log_user_ctrl_data) == 0 )
			{
				stat |= 0x10;
			}
			
			if(XY_Get_Ultra_Data(&_log_ultra_data, ULTRA_GET_ID_A) == 0)
		   	{
                           if(!ultra_queue_full(pq))
                           {
                                push_queue(pq,_log_ultra_data);
                                //printf("_log_ultra_data is %.4f\n",_log_ultra_data);
                           }
                           if(ultra_queue_full(pq))
                           {
                                ultra_calc(pq);
								Get_calced_Ultra(pq,&_calc_ultra_data);
                                queue_pop(pq);                            
                                //printf("_calc_ultra_data is %.4f\n",_calc_ultra_data);
                           }
			   	stat |= 0x20;
		   	}

			/* sd code should use OFFSET_GET_ID_B */
			if( XY_Get_Offset_Data(&_log_offset, OFFSET_GET_ID_B) == 0)
			{
				stat |= 0x40;

				get_log_offset_adjust(&_log_offset_adjust);
				stat |= 0x80;
			}

			
                              
			if( gettimeofday(&tv, NULL) != 0)
			{
				printf("ERROR get timestamp for log.\n");
				
			}
			tmlocal = localtime(&tv.tv_sec); 
        	dji_time=((float)cur_broadcast_data.time_stamp/600);
			
			sprintf(_wbuf, "%d-%d %d:%d:%d:%6ld;%6lf;",	1 + tmlocal->tm_mon,
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
			check_stat_store(stat, 0x10, _wbuf);
			check_stat_store(stat, 0x20, _wbuf);
			check_stat_store(stat, 0x40, _wbuf);
			check_stat_store(stat, 0x80, _wbuf);
		  
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

