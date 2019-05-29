#ifndef OBSTACLE_DETECTION_H_INCLUDED
#define OBSTACLE_DETECTION_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iostream>

#include <cmath>
#include "lane_recog.h"

using namespace cv;
using namespace std;

extern int maxArea;
extern int MaxArea;

extern int thresh;

extern int ObstacleFlag;

extern const int OBS_MAX_AREA;

extern Mat src_obs, src_gray, dst;
extern vector<Mat> channels;

void ObstacleDetection(int rows, int cols, Mat &frame);
//void ObstacleDetection_Init(int, void*);
//void ObstacleThreshold(int, void*);
#endif
