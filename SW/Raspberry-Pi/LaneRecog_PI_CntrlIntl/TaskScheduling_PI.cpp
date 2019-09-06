
//#include "blob.cpp"
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <raspicam/raspicam_cv.h>
//#include "decision_making.cpp"
#include "lane_recog.h"
#include "Sign_Reg.h"
#include "decision_making.h"
#include "obstacle_detection.h"

//using namespace cv;
//using namespace std;

   Mat my_image;

void *TaskSched10ms(void *arg)
{	while(1){
		trackingLogic();

		//ObstacleDetection(my_image.rows, my_image.cols, my_image);
		//imshow("BBoxes", my_image);

		usleep(50000);//send steering cmd every 100 ms	was 50000
	}
	return NULL;
	
}

/** @function main */
int main( int argc, char** argv )
{
   clock_t CurTime = clock();
   float deltaT = 0;
   pthread_t thread[1];
   int rc;
   //Mat my_image;

   



//Yue Sun Jan-9 2017, use camera.open and retrieve func instead of video cap
//attempting to improve speed

#if 0
	//TO DO Need to cal frame rate
	VideoCapture cap(0); //capture the video from webcam

	if ( !cap.isOpened() )  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}
	
	//2016-11-08 F.Dang Frame rate per second setting
	//cap.set(CV_CAP_PROP_FPS, 15);
	
	
	//Capture a temporary image from the camera
	cap.read(src); 

	//Create a black image with the size as the camera output
	Mat imgLines = Mat::zeros( src.size(), CV_8UC3 );;

	/// Load an image
	//src = imread( argv[1] );

	// Half image dimensions
	resize(src, src, Size(FRAME_WIDTH, FRAME_HEIGHT), 0,0);


#endif	

    //Yue Sun Jan 9 2017
    //Referencing Hongbo's code to use Camera, use our own window size	
    //set camera params
    raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
    Camera.set( CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    Camera.set( CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    Camera.grab();
    Camera.retrieve (src);
    CannyThreshold(0, 0, my_image);

   rc = pthread_create(&thread[0], NULL, TaskSched10ms, NULL);
   if (rc){
       printf("ERROR; return code from pthread_create() is %d\n", rc);
       exit(-1);
   }

    //imshow("bboxes", my_image);
    //Camera.retrieve (my_image);

    //Open camera
    cout<<"Opening Camera..."<<endl;
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//TEST MODE/////////////////////////////////////////////////////////////////////////////////////////
	//2016-11-08 F. Dang Debug mode enable
	if ( lane_reg_b_debug_md_enbl_s == DEBUG_MODE ){
		cout << "Image size: w=" << src.size().width << "\th=" << src.size().height << endl;
	}
	////////////////////////////////////////////////////////////////////////////////////////////

	if( !src.data )
	{ return -1; }

	/// Create a matrix of the same type and size as src (for dst)
	dst.create( src.size(), src.type() );

	/// Convert the image to grayscale
	cvtColor( src, src_gray, CV_BGR2GRAY );

	// Alternate: convert image to HSV and grab H
	cvtColor( src, src_hsv, CV_BGR2HSV);
	split( src_hsv, hsvChannels);

	//2016-11-08 intialize cal switch to enable debug mode
	lane_reg_b_debug_md_enbl_s = 1;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	//TEST MODE/////////////////////////////////////////////////////////////////////////////////////////
	//2016-11-08 F. Dang Debug mode enable
	if ( lane_reg_b_debug_md_enbl_s == DEBUG_MODE ){
		
		
		/// Create a window
		namedWindow( window_name, CV_WINDOW_NORMAL );
		//namedWindow( options_window, CV_WINDOW_NORMAL);
		/*
		/// Create a Trackbar for user to enter threshold
		createTrackbar( "Min Threshold", options_window, &lowThreshold, max_lowThreshold, CannyThreshold );
		// to be added for min line length and max gap length
		createTrackbar( "Ratio", options_window, &ratio, max_ratio, CannyThreshold );
		//createTrackbar( "Use Hue instead of intensity", options_window, &useHsv, 1, CannyThreshold);
		createTrackbar( "PHLenable", options_window, &useHoughP, 1, CannyThreshold);
		createTrackbar( "(P)HTThreshold", options_window, &houghThresh, max_houghThresh, CannyThreshold);
		createTrackbar( "PHL min line length", options_window, &min_Line_Length, max_minlineLength, CannyThreshold);
		createTrackbar( "PHL max line gap", options_window, &max_line_gap, max_maxlineGap, CannyThreshold);

		createTrackbar( "Single Line Detect MD cal learn", options_window, &phl_sw_sngl_cal_lrn_enbl_s, 1, CannyThreshold);

		createTrackbar( "Min Left Lane Theta", options_window, &minLeftThetaDegrees, 90, Lane_Reg_Init);
		createTrackbar( "Max Left Lane Theta", options_window, &maxLeftThetaDegrees, 90, Lane_Reg_Init);
		createTrackbar( "Min Right Lane Theta", options_window, &minRightThetaDegrees, 180, Lane_Reg_Init);
		createTrackbar( "Max Right Lane Theta", options_window, &maxRightThetaDegrees, 180, Lane_Reg_Init);

		createTrackbar("Lower GO H", options_window, &filtLowHGO, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Upper GO H", options_window, &filtHighHGO, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Lower GO S", options_window, &filtLowSGO, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Upper GO S", options_window, &filtHighSGO, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Lower GO V", options_window, &filtLowVGO, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Upper GO V", options_window, &filtHighVGO, FILT_HSV_MAX, Sign_Reg_Init);

		createTrackbar("Lower STOP H", options_window, &filtLowHSTOP, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Upper STOP H", options_window, &filtHighHSTOP, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Lower STOP S", options_window, &filtLowSSTOP, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Upper STOP S", options_window, &filtHighSSTOP, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Lower STOP V", options_window, &filtLowVSTOP, FILT_HSV_MAX, Sign_Reg_Init);
		createTrackbar("Upper STOP V", options_window, &filtHighVSTOP, FILT_HSV_MAX, Sign_Reg_Init);
		*/
		//createTrackbar("Min Area", options_window, &MinArea, FILT_MIN_AREA_MAX, Sign_Reg_Init);

		//createTrackbar("ObsArea", options_window, &MaxArea, OBS_MAX_AREA, ObstacleDetection_Init);

	}
	//Need to load default cal value when it is under debug mode


	// Initialize min Theta
	Lane_Reg_Init(0,0);

  while(true){
	//Yue Sun Jan-9 2017, try camera to improve speed
	#if 0
	     bool bSuccess = cap.read(src); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }
	#endif
   	Camera.grab();
    	Camera.retrieve (src);
	if(src.empty()){break;}
	waitKey(1);

		// 2016-07-29 Region of interest, crop region of interest for lane detection area
		//ROI (tx, ty, width, height)
		//Rect ROI(0,ROI_Y,src.size().width,ROI_Height);
		//src = src(ROI);

		
		cout << "Delta T: " << deltaT << endl;


		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		//TEST MODE/////////////////////////////////////////////////////////////////////////////////////////
		//2016-11-08 F. Dang Debug mode enable
		if ( lane_reg_b_debug_md_enbl_s == DEBUG_MODE ){
			
			/// Show the image
			imshow("Original", src); //show the original image
			//imshow("HSV", src_hsv); // show Sign Recognition HSV image
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
		// Alternate: convert image to HSV and grab H
		//Yue Sun: comment out HSV to improve speed Jan-6-2017
		Rect ROI(0,ROI_SIGN,src.size().width,ROI_SIGN_HEIGHT);
		src_sign = src(ROI);
		
		cvtColor( src_sign, src_hsv, CV_BGR2HSV);
		split( src_hsv, hsvChannels);
		


		SignReg(src_hsv,src_sign);
		//lane_reg_ct_x_proc(0,0);	
	        CannyThreshold(0, 0, my_image);
		//ObstacleDetection(my_image.rows, my_image.cols, my_image);
		imshow("contour", my_image);





//Yue Sun Jan 9 2017 - Use 50ms pthred to manage send cmd, disable 100ms task in while loop
#if 0
		if( (deltaT = (clock()-CurTime)/double(CLOCKS_PER_SEC)) > TaskPeriod){

					//cout << "Cross_X: " << x_lane_cross << endl;
           		//cout << "Cross_Y: " << y_lane_cross << endl;
			trackingLogic();
				cout << endl;
			CurTime = clock();
		}
#else
	deltaT = (clock()-CurTime)/double(CLOCKS_PER_SEC);
        CurTime = clock();
#endif
	
		/// Wait until user exit program by pressing a key
	    if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break; 
        }

  }

  return 0;
}
