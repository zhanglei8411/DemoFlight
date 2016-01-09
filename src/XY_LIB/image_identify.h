#ifndef __IMAGE_IDENTIFY_H__
#define __IMAGE_IDENTIFY_H__

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "getTargetLoc.hpp"
#include "opencv.hpp"
#include "../DJI_LIB/DJI_Pro_App.h"



typedef struct
{
	float x;
	float y;
	float z;
}Offset;


int XY_Get_Offset_Data(Offset *_data);
int XY_Start_Capture(void);
int XY_Stop_Capture(void);
int XY_Capture_Setup(void);

#endif
