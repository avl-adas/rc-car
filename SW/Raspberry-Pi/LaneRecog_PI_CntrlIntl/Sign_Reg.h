#ifndef SIGN_REG_H_INCLUDED
#define SIGN_REG_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iostream>

#include <cmath>
// #include <vector>


#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"
//#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

extern const int FILT_HSV_MAX;
extern const int FILT_MIN_AREA_MAX;

/*
// Key codes
const int KEY_ESC   = 27;
const int KEY_ENTER = 10;
const int KEY_ENTER_WINDOWS = 13;
*/




// Global coordinate of moments
extern int blob_currX;
extern int blob_currY;



// Default settings
extern int filtLowHGO;
extern int filtHighHGO;

extern int filtLowSGO;
extern int filtHighSGO;

extern int filtLowVGO;
extern int filtHighVGO;

extern int filtLowHSTOP;
extern int filtHighHSTOP;

extern int filtLowSSTOP;
extern int filtHighSSTOP;

extern int filtLowVSTOP;
extern int filtHighVSTOP;

extern int minArea;
extern int MinArea;
extern int SignFlag;
extern int Sign_Flag;

extern const int ROI_SIGN;
extern const int ROI_SIGN_HEIGHT;

// logic for latching on stop light - 05/31/2019
extern const int thresh_1;
extern const int thresh_2;

extern int ct_red_light;
extern int ct_non_red_light;

extern Mat src_sign;

void SignReg(Mat frame_hsv,Mat frame);
void Sign_Reg_Init(int, void*);
#endif
