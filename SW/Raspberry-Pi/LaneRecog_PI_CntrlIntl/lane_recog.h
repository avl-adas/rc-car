#ifndef LANE_RECOG_H_INCLUDED
#define LANE_RECOG_H_INCLUDED

#include <string>
#include <fstream>
#include <omp.h>
#include "decision_making.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <cmath>
#include <stdio.h>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */

/// Global variables
////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
//Constants//////////////////////////////////////////////////////////////////
//used for tracker bar
extern int const max_lowThreshold;
extern int const max_ratio;
extern int const max_kernelSize;
extern int const max_kernelFilterSize;
extern int const max_houghThresh;
extern int const max_minlineLength;
extern int const max_maxlineGap;
extern int minThetaDegrees;
extern float minTheta;
//Y direction section threshold Need to optimize cal and enable in adjustable bar
//TO BE FIXED: self define section number
//left right lane idx 
extern int const LEFT_LANE;
extern int const RIGHT_LANE;

extern int const MAX_LANE_NUM;

extern int const CENTER_COR_X;

extern int LaneFlag1;
extern int LaneFlag2;

/*
//section idx
extern int const SECT_1;
extern int const SECT_2;
extern int const SECT_3;
extern int const SECT_4;
//2016-08-28 Phougline section threshold
extern int const SECT0_THRSH;
extern int const SECT1_THRSH;
extern int const SECT2_THRSH;
extern int const SECT3_THRSH;
extern int const SECT4_THRSH;
*/

//detected line point coordinate idx
extern const int PT1_X;
extern const int PT1_Y;
extern const int PT2_X;
extern const int PT2_Y;

extern int const MAX_PT_NUM;

//2016-07-30 ROI range define 
//ROI_Y - region of interest start Y axis
extern int const FRAME_WIDTH;
extern int const FRAME_HEIGHT;
// Adjust region of interest
extern int const ROI_Y;
extern int ROI_Height;

extern char* window_name;
extern char* options_window;

//2016-11-08 Debug Mode cal switch
extern int const DEBUG_MODE;


//////////////////////////////////////////////////////////////////////////////////////////////////
//Calibration/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//Preprocessing cal
extern int kernelFilterSize;
extern int useHsv;

//edge detection cal
extern int lowThreshold;
extern int ratio;
extern int kernel_size;


//Hougline calibration
extern int useHoughP;
extern int houghThresh;
extern int min_Line_Length;
extern int max_line_gap;
////////////////////////////////////////////////////////
extern int minLeftThetaDegrees;
extern int maxLeftThetaDegrees;
extern int minRightThetaDegrees;
extern int maxRightThetaDegrees;


extern int const MAX_SECT_NUM;
//Calibration value when only left lane detected, each section delta
extern int phl_thrsh_sngl_cal_lrn_s;// threshold to determine if center point land in CENTER_COR_X region
//TO DO: Initial Value need to calibrate based on actual road condition
//TO DO: Need to be global and determine reasonable intial value
extern float phl_pt_intc_x_2_ct_del_lrn[];//delta between intersection point (left & reference) and frame center x for bottom section each section - calibratable indx with sect_i
//TO DO: Need to add to tracker bar
extern int phl_sw_sngl_cal_lrn_enbl_s;//TRUE enable sigle line detect mode cal learn

//2016-11-08 F. Dang Debug mode switch signal
extern int lane_reg_b_debug_md_enbl_s;

////////////////////////////////////////////////////////////////////////////////////////////////////
//Signal///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
extern Mat src, src_gray;
extern Mat dst, detected_edges;

extern Mat src_hsv;
extern vector<Mat> hsvChannels;

//Section reference line y = phl_thrsh_y_sect
extern int phl_thrsh_y_sect[];
//section reference line in y direction
extern float phl_ln_ref_y[];
//section detected center point
//TO DO: Need to be global
extern float phl_pt_x_sect_ct[];// when both lane detected
extern float phl_pt_x_sect_ct_lft_est[];// when only left lane detected
extern float phl_pt_x_sect_ct_rt_est[];// when only right lane detected

// TO be optimized
//2016-07-01 F. Dang: Set Flag for valid result
extern int lane_reg_flag_det;
extern float lane_reg_buf[];
extern int lane_reg_buf_index;

extern float lane_reg_flt_x;
extern float lane_reg_flt_x2;
///////////////////////////////////////////////////
extern float x_lane_cross;
extern float y_lane_cross;

extern float Lane_Mod_Gn;



void CannyThreshold(int, void*, Mat &frame);
void Lane_Reg_Init(int, void*);
void lane_reg_ct_x_proc(int, void*);


#endif
