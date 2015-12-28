#ifndef __IMAGE_IDENTIFY_H__
#define __IMAGE_IDENTIFY_H__

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "getTargetLoc.hpp"
#include "opencv.hpp"


typedef struct
{
	float x;
	float y;
	float z;
}Offset;


void clear_refresh_flag(void);
int read_refresh_flag(void);
Offset get_stored_offset_block(void);
int XY_Ctreate_Capture_and_Identify_Thread(void);

#endif
