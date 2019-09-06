#include "obstacle_detection.h"
//src is a global variable that already defined

int maxArea = 150;
int MaxArea = 150;

//int thresh = 10;

int ObstacleFlag= 0; // 0 for Go , 1 for Stop

//Mat src_obs;
//vector<Mat> channels;

const int OBS_MAX_AREA = 200000000;

/*
void ObstacleDetection_Init(int, void*)
{

    MaxArea = maxArea;

}
*/

void ObstacleDetection(int rows, int cols, Mat &obs_frame)
{
    int largest_area=0;
    int largest_contour_index=0;
    int area_max = 0;
    Rect bounding_rect;

    //Mat *procSrc = *obs_frame;

    Scalar color(255,255,255);  // color of the contour

    //Mat thr(obs_frame.rows, obs_frame.cols, CV_8UC1);

    Mat dst(rows, cols, CV_8UC1, Scalar::all(0));

    //Mat mask = masked_image();
    //bitwise_and(thr, mask, thr);

    
    vector< vector<Point> > contours; // Vector for storing contour
    //vector< vector<Point> > contours0;
    vector< Vec4i > hierarchy;

    //cvtColor(obs_frame, thr, CV_BGR2GRAY);

    //threshold(thr, thr, 25, 255, THRESH_BINARY);

    //imshow("debug4", obs_frame);

    //threshold(obs_frame, obs_frame, 45, 255, THRESH_BINARY);

    //imshow("debug5", obs_frame);

    findContours(obs_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    
    // iterate through each contour.
    //cout << "Number of contours found:" << contours.size() << endl;
    //cout << "area of the first contour:" << contourArea(contours[1], false) << endl;
    for( size_t i = 0; i < contours.size(); i++ )
    {
        	
	//approxPolyDP(contours[i], contours0[i], 3, true);
	//0.01*arcLength(contours[i], true)
	//Rect r = boundingRect(contours[i]);

	//if (hierarchy[i][2] < 0)
	//{
		//  Find the area of contour
		//rectangle(some_image, Point(r.x, r.y), Point(r.x+r.width, r.y+r.height), Scalar(0, 255, 0), 2, 8, 0);
        double a=contourArea(contours[i],false);
	double length = arcLength(contours[i], true);
        if(a > largest_area){
		            	largest_area=a;
        			// Store the index of largest contour
        			largest_contour_index=i;
				bounding_rect = boundingRect(contours[i]);
        		    }
	area_max += a;
	
    	//}
    }

    drawContours(dst, contours, -1, color, CV_FILLED, 8, hierarchy);
    //rectangle(src, bounding_rect, Scalar(0, 255, 0), 2, 8, 0);
        
    //cout << "all area sum:" << area_max << endl;
    cout << "largest area:" << largest_area << endl;

    if(largest_area > 500)
    {

	ObstacleFlag = 1;
	//cout << "Obstacle ahead" << endl;

    }
    else
    {

	ObstacleFlag = 0;
	//cout << "Cleared to go" << endl;

    }

    //imshow("Original frame", obs_frame);
    //imshow("Largest Contour", dst);
    //imshow("bboxes", src);
    //imshow("Threshold", );
    

}



 
