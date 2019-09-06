// Canny Edge Detector OpenCV tutorial: http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html

#include "lane_recog.h"

#include <tuple>

/////////////////////////////////////////////////////////////////////////////
//Constants//////////////////////////////////////////////////////////////////
//Used for tracker bar
int const max_lowThreshold = 300;
int const max_ratio = 300;
int const max_kernelSize = 10;
int const max_kernelFilterSize = 10;
int const max_houghThresh = 200;
int const max_minlineLength = 70;
int const max_maxlineGap = 300;

int LaneFlag1 = 0;
int LaneFlag2 = 0;

//Y direction section threshold Need to optimize cal and enable in adjustable bar
//TO BE FIXED: self define section number
//left right lane idx 
int const LEFT_LANE = 0;
int const RIGHT_LANE = 1;
int const MAX_LANE_NUM = 2;

//detected line point coordinate idx
int const PT1_X = 0;
int const PT1_Y = 1;
int const PT2_X = 2;
int const PT2_Y = 3;
int const MAX_PT_NUM = 4;

/*
//section idx
int const SECT_1 = 0;
int const SECT_2 = 1;
int const SECT_3 = 2;
int const SECT_4 = 3;
//2016-08-28 Phougline section threshold - calibration
int const SECT0_THRSH = 0;//section i threshold
int const SECT1_THRSH = 100;//section i threshold
int const SECT2_THRSH = 140;//section i threshold
int const SECT3_THRSH = 170;//section i threshold
int const SECT4_THRSH = 200;//section i threshold
*/

int const MIN_SECT_NUM = 0;//always be 0 refer to MAX_SECT_NUM

int const MAX_LOOP_NUM = 4;//maximum loop to memorize phl_pt_x_sect_ct[sect_i]

char* window_name = "Edge Map";
char* debug = "debug 1";
char* debug2 = "debug 2";
char* debug3 = "debug 3";
char* debug_orig = "debug_orig";
char* window_name_line = "Line_Detected";
char* options_window = "Options";

//2016-08-28 ROI range define 
//ROI_Y - region of interest start Y axis
int const FRAME_WIDTH = 320;//600;
int const FRAME_HEIGHT = 240;//400;
// Adjust region of interest
int const ROI_Y = FRAME_HEIGHT/2;//335;
int ROI_Height = FRAME_HEIGHT/2;//65;
int ROI_2_Height = FRAME_HEIGHT/6;

int const CENTER_COR_X = FRAME_WIDTH/2;//Center of Image Frame
int const CENTER_COR_Y = ROI_Height/2;// Center of ROI_Y

//2016-11-08 Debug Mode cal switch
int const DEBUG_MODE = 1;

////////////////////////////////////////////////////////////////////////////////////////////////////
//Calibration parameter//////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

//2016-11-08 F. Dang Debug mode switch signal
int lane_reg_b_debug_md_enbl_s = 0;

//Preprocessing cal
int kernelFilterSize = 5;//Preprocessing filter size - Guassian Filter
int useHsv = 0;

//edge detection cal
int lowThreshold = 150;//200;//Edge detection threshold
int ratio = 175;//230;// Edge detection ratio threshold
int kernel_size = 3;//Edge detection

//Hougline calibration
int useHoughP = 0;// Phoughline enable switch
int houghThresh = 45;// PHL HL threshold
//2016-08-28 F. Dang Use for Phougline 
//TO DO: Need to add to calibration tracker bar
int min_Line_Length = 3;// PHL min line length threshold - Should be double in PHL
int max_line_gap = 180;//PHL max line gap threshold - Should be double in PHL

// Angle Threshold defination - Degree & Rad - Need to be calibrated
// Default angle calibration value may need more implementation for robustness 
int minLeftThetaDegrees = 2; // was 5
int maxLeftThetaDegrees = 75; // was 85
int minRightThetaDegrees = 115; // was 95
int maxRightThetaDegrees = 178; // was 175

float phl_rho_array[15];
float phl_theta_array[15];

//2016-09-02 F. Dang multi section calibratable
int const MAX_SECT_NUM = 4;//must be int > 1 and could be divided by ROI_HEIGHT without any residual

//Calibration value when only left lane detected, each section delta
int phl_thrsh_sngl_cal_lrn_s = 2;// Tolerance threshold to determine if center point land in CENTER_COR_X region
//TO DO: Initial Value need to calibrate based on actual road condition
//TO DO: Need to be global and determine reasonable intial value
float phl_pt_intc_x_2_ct_del_lrn[MAX_SECT_NUM];//delta between intersection point (left & reference) and frame center x for bottom section each section - calibratable indx with sect_i
//float phl_pt_intc_x_2_ct_del_ct[4] = { 47, 226, 256 , 264 };//Compare with learned result
//TO DO: Need to add to tracker bar
int phl_sw_sngl_cal_lrn_enbl_s = 1;//TRUE enable sigle line detect mode cal learn

float phl_coef_sect_trans[MAX_SECT_NUM];//section transition coef

float phl_coef_sect_trans_s = 1.01;

//int phl_thrsh_sngl_md_updt_s = 2;// calibration for signle line detection mode update intersct point value enable



//TO DO: need to clearfy usage
//Used for saturation of the final out put of dynamic center point
float x_lane_cross = 0;
float y_lane_cross = 0;
const float x_lane_cross_max = FRAME_WIDTH - 50;//max saturation
const float y_lane_cross_max = 500;
const float x_lane_cross_min = 50;
const float y_lane_cross_min = 100;

const float max_rho_det = 10.0F;

const int BUF_SIZE = 50;


int lane_width_far_lft = FRAME_WIDTH/4;
int lane_width_far_rt = FRAME_WIDTH/4;
int lane_width_near = FRAME_WIDTH/4;
int lane_width_far_lft_ROI2 = FRAME_WIDTH/6;
int lane_width_far_rt_ROI2 = FRAME_WIDTH/7;

int del_min_dist_sgl_ln_s = 100;
int del_min_dist_sgl_ln_ROI2_s = 100;

unsigned int lane_enum_ROI1 = 0; 	// 0 - No lane or Double lane decided by ROI1, 1 - Right, 2 - left
//Yue Sun Jan-9-2017
//Calculated x cor could be out of frame width, limiting width to filter outer lanes
int lane_width_near_min = -FRAME_WIDTH/5;
int lane_width_near_max =  FRAME_WIDTH*6/5;

int lane_width_near_min_roi2 = 0;
int lane_width_near_max_roi2 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////
//Signal///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

Mat src, src_gray, src_hls;
Mat mask_hls;
//Mat src_tmp;
Mat src_tmp2, src_tmp3;
Mat dst, detected_edges, detected_edges_orig, detected_edge_up_roi, detected_edge_obs_roi, detected_edge_low_roi;

Mat src_hsv;
vector<Mat> hsvChannels;


float min_Left_Theta = 0;//range of left theta in raduis - convert from minLeftThetaDegrees
float max_Left_Theta = 0;//range of left theta in raduis - convert from maxLeftThetaDegrees
float min_Right_Theta = 0;//range of left theta in raduis - convert from minRighttThetaDegrees
float max_Right_Theta = 0;//range of left theta in raduis - convert from maxRightThetaDegrees


//2016-09-02 F. Dang section threshold and height in y direction
int SECT_HEIGHT = ROI_Height/MAX_SECT_NUM;


//Valid lane detection flag idex with [LEFT_LANE] [RIGHT_LANE]
bool phl_b_vld_ln_det[MAX_LANE_NUM] = {0, 0};
bool phl_b_vld_ln_det_sec[MAX_SECT_NUM][MAX_LANE_NUM];

//Section reference line y = phl_thrsh_y_sect
int phl_thrsh_y_sect[(MAX_SECT_NUM + 1)]; 
//section reference line in y direction
float phl_ln_ref_y[MAX_SECT_NUM];
//section detected center point
//TO DO: Need to be global
float phl_pt_x_sect_ct[MAX_SECT_NUM];// when both lane detected
float phl_pt_x_sect_ct_lft_est[MAX_SECT_NUM];// when only left lane detected
float phl_pt_x_sect_ct_rt_est[MAX_SECT_NUM];// when only right lane detected

float phl_pt_x_ct_loop[MAX_SECT_NUM][MAX_LOOP_NUM];
bool phl_b_vld_ln_det_sec_loop[MAX_SECT_NUM][MAX_LOOP_NUM];//flag indicate valid detection idx with sect_i and loop_j

int phl_ptr_curt_loop = 0;//pointer for current loop

float phl_pt_x_sect_rt_line[BUF_SIZE];//buffer to record right lane intersect point with reference point
float phl_pt_x_sect_lft_line[BUF_SIZE];//buffer to record left lane intersect point with reference point

float lane_reg_buf[BUF_SIZE];
int lane_reg_buf_index = 0;

float lane_reg_flt_x = CENTER_COR_X;
float lane_reg_flt_x2 = CENTER_COR_X;

float lane_reg_flt_x_old = CENTER_COR_X;
float lane_reg_flt_x2_old = CENTER_COR_X;

float lane_assignment_midpt2 = 0.0f;




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////USED FOR HOUGHLINE, to be removed?//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//2016-09-01 F. Dang need to utilize for Kalman Filter
//2016-07-01 F. Dang: Set Flag for valid result
int lane_reg_flag_det = 1;


/////////////////////////////////////////////////////////////////////////////////////
//2016-07-01 F. Dang: Set Flag for valid result
int const LANE_REG_VALID = 1;
int const LANE_REG_NOT_VALID = 0;
/////////////////////////////////////
float a=1, b=0, c=1, d=0;// used for drawing detected houghline
// Center of the frame - Hougline used only
float Coor_X = 300;// for Houghline used only to calculate dynamic center point X of two lane
float Coor_Y = 200;// for Houghline used only to calculate dynamic center point Y of two lane
//2016-08-05 F.Dang Center Cor Strategy
int const LANE_REG_D_HRZ = 25;
const int LANE_REG_D_HRZ_BUF_SIZE = 4;
float lane_reg_d_hrz_buf[LANE_REG_D_HRZ_BUF_SIZE][2];
float Lane_Mod_Gn = 1.0F;

//Effort for Adaptive ROI
// Yue Sun and Ankit G. date: 05/28/2019
const float del_x_vals[11] = {0.5, 0.5, 0.4, 0.25, 0, 0, 0, -0.25, -0.4, -0.5, -0.5};
const int steer_vals[11] = {-30, -20, -10, -5, -2, 0, 2, 5, 10, 20, 30};


float interpol_1d(const int *array_x, const float *array_y, unsigned int size, int value){

	unsigned int index = 0; //value/(array_x[size-1]/(size-1)); /*locate index of value, use integer math to cut residue*/

	while (array_x[index] < value && index < (size-1))

	{

		index++;

	}

	if ( index == 0 ){
		return array_y[0]; /*No extropolation beyond the min*/
	}
	else if( index < (size-1)){
		return (    array_y[index] + (array_y[index+1]-array_y[index])/(array_x[index+1]-array_x[index])*(value-array_x[index])   ); /*interpolation*/
	}
	else{
		return array_y[size-1];/*No extropolation beyond the max*/
	}
	
}

float del_x_mult = 0.0f;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*float Calc_average(int sample_number, int idx, float Array[])
{
	float output = 0;
	float Sum = 0;
	for(int i = 0; i < sample_number; i++)
	{
		Sum = Sum + Array[idx - i];
	}
	output = Sum/sample_number;
	return output;
}*/



//17-08-02 Ankit mask function to crop ROI
//19-03-27 Yue Sun expand lower ROI for new figure 8 lane
Mat masked_image(){

	Mat mask_source = Mat::zeros( detected_edges.size(), detected_edges.type() ) ;
        //resize(mask_source, mask_source, Size(FRAME_WIDTH, FRAME_HEIGHT), 1,1);	
	Point mask_pt1(max((del_x_mult/4)*FRAME_WIDTH,0.0f), 3*FRAME_HEIGHT/4); 
	Point mask_pt2(max((FRAME_WIDTH/16 + (del_x_mult/2)*FRAME_WIDTH),0.0f),ROI_Height); // was 1/8
	Point mask_pt3(min((15*(FRAME_WIDTH)/16 + (del_x_mult/2)*FRAME_WIDTH), float(FRAME_WIDTH)),ROI_Height); // was 7/8
	Point mask_pt4(min((FRAME_WIDTH + (del_x_mult/4)*FRAME_WIDTH), float(FRAME_WIDTH)), 3*FRAME_HEIGHT/4);
	//define point array to store all contour points
	Point mask_pt_ary[1][4];
	
	mask_pt_ary[0][0] = mask_pt1;
	mask_pt_ary[0][1] = mask_pt2;
	mask_pt_ary[0][2] = mask_pt3;
	mask_pt_ary[0][3] = mask_pt4;
 	
	const Point* ppt[1] = { mask_pt_ary[0] };
	int npt[] = { 4 };

	fillPoly(mask_source, ppt, npt,1, Scalar(255,255,255),8);
	return mask_source;
	
}
//19-03-27 Yue Sun expand upper ROI for new figure 8 lane
Mat masked_image_roi2(){

	Mat mask_source = Mat::zeros( detected_edges.size(), detected_edges.type() ) ;
        //resize(mask_source, mask_source, Size(FRAME_WIDTH, FRAME_HEIGHT), 1,1);	
	Point mask_pt1(max((FRAME_WIDTH/16 + (del_x_mult/2)*FRAME_WIDTH),0.0f),ROI_Height); 
	Point mask_pt2( max((FRAME_WIDTH/6 + del_x_mult*1.2f*FRAME_WIDTH), 0.0f), ROI_2_Height);
	Point mask_pt3(min((5*(FRAME_WIDTH)/6 + del_x_mult*1.2f*FRAME_WIDTH), float(FRAME_WIDTH)),ROI_2_Height);
	Point mask_pt4(min((15*(FRAME_WIDTH)/16 + (del_x_mult/2)*FRAME_WIDTH), float(FRAME_WIDTH)),ROI_Height);
	//define point array to store all contour points
	Point mask_pt_ary[1][4];
	
	mask_pt_ary[0][0] = mask_pt1;
	mask_pt_ary[0][1] = mask_pt2;
	mask_pt_ary[0][2] = mask_pt3;
	mask_pt_ary[0][3] = mask_pt4;
 	
	const Point* ppt[1] = { mask_pt_ary[0] };
	int npt[] = { 4 };

	fillPoly(mask_source, ppt, npt,1, Scalar(255,255,255),8);
	return mask_source;
	
}


Mat masked_image_roi3(){

	Mat mask_source = Mat::zeros( detected_edges.size(), detected_edges.type() ) ;
        //resize(mask_source, mask_source, Size(FRAME_WIDTH, FRAME_HEIGHT), 1,1);	
	Point mask_pt1(0.2*FRAME_WIDTH,FRAME_HEIGHT); 
	Point mask_pt2(0.4*FRAME_WIDTH,ROI_2_Height);
	Point mask_pt3(0.6*FRAME_WIDTH,ROI_2_Height);
	Point mask_pt4(0.8*FRAME_WIDTH,FRAME_HEIGHT);
	//define point array to store all contour points
	Point mask_pt_ary[1][4];
	
	mask_pt_ary[0][0] = mask_pt1;
	mask_pt_ary[0][1] = mask_pt2;
	mask_pt_ary[0][2] = mask_pt3;
	mask_pt_ary[0][3] = mask_pt4;
 	
	const Point* ppt[1] = { mask_pt_ary[0] };
	int npt[] = { 4 };

	fillPoly(mask_source, ppt, npt,1, Scalar(255,255,255),8);
	return mask_source;
	
}

//Mat mask_white_lines( Mat my_im ){

//	Mat mask = Mat::zeros(my_im.size(), my_im.type());

	//was 145, 145, 145
	//was 200, 200, 200 and 255, 255, 255
	
//	return mask;
//}

	

/*pass in array p, return the max index and value*/
void max_find(float *p, float *q, int *ind, float *val){

	unsigned int i = 0;
	for(i=0;i<15;i++){
		if (i<1){
			*ind = i;
			*val = p[i];  
		}
		else{
			if ((p[i]>*val) && ((q[i]< max_Right_Theta && q[i]> min_Right_Theta) || (q[i]< max_Left_Theta && q[i]> min_Left_Theta))){
				*val = p[i];
				*ind = i;
			}
		}

	}

}
void min_find(float *p, float *q, int *ind, float *val){

	unsigned int i = 0;
	for(i=0;i<15;i++){
		if (i<1){
			*ind = i;
			*val = p[i];  
		}
		else{
			if ((p[i]<*val) && ((q[i]< max_Right_Theta && q[i]> min_Right_Theta) || (q[i]< max_Left_Theta && q[i]> min_Left_Theta))){
				*val = p[i];
				*ind = i;
			}
		}

	}

}

//Aug 10 2016 M. Heydari Convert Cartesian to Ploar Coordinate system
tuple < float,float >  pointsToRhoTheta(float x1,float x2,float y1,float y2)
{

   float cvTheta=0;
   float cvRho=0;
   float k=0;
   if (x2-x1!=0) {
      k=(y2-y1)/(x2-x1);
   } else {
      k=99999;
   }

   float xSlope,ySlope,m;
   m=y1-k*x1;

   xSlope=-m/(k+(1/k));
   ySlope=m/(pow(k,2)+1);

   if (std::atan2((ySlope),(xSlope))<0) {
      cvTheta=std::atan2((ySlope),(xSlope))+CV_PI;
      cvRho=-std::sqrt(pow(xSlope,2)+pow(ySlope,2));
   } else {
      cvTheta=std::atan2((ySlope),(xSlope));
      cvRho=std::sqrt(pow(xSlope,2)+pow(ySlope,2));
   }


   float lineRho=cvRho;
   float lineTheta=cvTheta;

   return make_tuple(lineRho,lineTheta);
}


//Aug 28 2016 F. Dang calculate slope and offset for line
//Note: input argument seqence 
tuple < float,float >  pointsTolineEq(float x1,float y1,float x2,float y2)
{

   float k=0;
   if (x2-x1!=0) {
      k=(y2-y1)/(x2-x1);
   } else {
      k=99999;
   }

   float m;
   m=y1-k*x1;

   return make_tuple(k,m);

}

/////////////////////////////////


void CannyThreshold(int, void*, Mat &ptr)
{


	Mat* procSrc;


	//Yue Sun Jan-08 2017, Added to extract ROI of src first, attempting to speed up 
	/// Convert the image to grayscale
	//src = src(ROI);

	//inRange(src, Scalar(0,0,212), Scalar(131,255,255), mask);


	Mat mask1 = Mat::zeros(src.size(), CV_8UC1);
	//Mat res_tmp = Mat::zeros(src_gray.size(), src_gray.type());
	cvtColor(src, src_hls, CV_BGR2HLS);
	inRange(src_hls, Scalar(0, 200, 0), Scalar(180, 255, 255), mask1);
	//bitwise_and(src, src, src_tmp2, mask1);
	//cvtColor(src_tmp, res_tmp, CV_BGR2GRAY);

	procSrc = &mask1;

	//imshow("bitwise_and", src_tmp2);
	imshow("mask", mask1);

	
	/// Reduce noise with a kernel 3x3
	// ACTION: use Guassian Filter/ Median Filter instead of basic blur
	//2016-08-29 Use Gaussian Filter
	//blur( *procSrc, detected_edges, Size(kernelFilterSize,kernelFilterSize) );
	//GaussianBlur(InputArray src, OutputArray dst, Size ksize, double sigmaX, double sigmaY=0, int borderType=BORDER_DEFAULT )
	
	//Yue Sun Jan-08 2017, comment out to test speed - no significant difference
	GaussianBlur(*procSrc, detected_edges, Size(kernelFilterSize,kernelFilterSize), 0, 0);

	imshow("gaussian", detected_edges);
	
        //ROI for edge detection
        //Yue Sun Jan-08 2017, comment out since procSrc has ROI cut
	//detected_edges = detected_edges(ROI);

	/// Canny detector
	/// Was Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
	//Canny( *procSrc, detected_edges, lowThreshold, ratio, kernel_size );
	Canny( detected_edges, detected_edges, lowThreshold, ratio, kernel_size );
	imshow(debug, detected_edges);

	del_x_mult = interpol_1d(steer_vals, del_x_vals, sizeof(steer_vals), CntlCom[0]-4);

	cout << "delta x" << del_x_mult << endl;

	Mat mask = masked_image();
	Mat mask_2 = masked_image_roi2();
	Mat mask_3 = masked_image_roi3();
	//Mat mask_det_edge;

	bitwise_and(detected_edges,mask,detected_edge_up_roi);
	imshow(debug2, detected_edge_up_roi);

	//August 04: Implementing ROI Mask function
	
	

	//Canny( detected_edges, detected_edges_3, newlowThreshold, ratio, kernel_size );

	bitwise_and(detected_edges, mask_3, detected_edge_obs_roi);
	
	ptr = detected_edge_obs_roi;

	/// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);


	//2017_01-03 for houghline two lane detection flag
	bool hl_b_vld_dbl_ln_det = 0;// intialized as false indicate no double lane is detected
	//resize(dst, dst, Size(FRAME_WIDTH, FRAME_HEIGHT), 0,0);

	//Yue Sun 10/23/2016 - Apply rate limit on rho to fix line jumping
	float rho, theta, rho_left, theta_left, rho_right, theta_right;
	int index_left,index_right;
	/*The whole block wrapped by {} is designed for ROI lane detection and center return of ROI 1 - Yue Sun Oct 8, 2017*/
		//Used for houghline////////////////////////////////////////////////
		//Record Left/Right theta

		/*Used or Not - Rahul Shetty*/
		float Left_Lane_Theta = 0;
		float Right_Lane_Theta = 0;
		// Log old theta for left & right - start from 90 degree: Right Lane search largest theta value, Left Lane search smallest theta value
		float Old_Left_Theta = 90;
		float Old_Right_Theta = 90;

		// Log Right & Left Rho - Default value determined by frame size
		//e.g. 600 x 400, default Left Rho is 1 and Right Rho will be 600
		// Left Rho default value is
		float Left_Lane_Rho = 1;
		float Right_Lane_Rho = 600;
		//////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////

		//2017 - 01 - 03 Reference to Hongbo's code
		float hl_pt_x_bottom_thrsh[2] = {FRAME_WIDTH,0};
		int ct_num_lft_det_lanes = -1;
		int ct_num_rt_det_lanes = -1;

		Point Rpt1(0,ROI_Height);
        Point Lpt1(0,ROI_Height);
        Point Rpt2(0,FRAME_HEIGHT);
        Point Lpt2(0,FRAME_HEIGHT);
		Point Midpt1(CENTER_COR_X,ROI_Height);
        Point Midpt2(CENTER_COR_X,ROI_Height);
	Point Old_lt_pt(0,0), Old_rt_pt(0,0);


		//2017-01-03 hongbo's reference
		int houghthresh_adj_hi_s = houghThresh;

		int houghthresh_adj_delta_s = 5;

		int houghthresh_adj_cnt_s = 3;
		int houghthresh_adj_low_s = houghthresh_adj_hi_s - houghthresh_adj_delta_s*houghthresh_adj_cnt_s;
		int lane_mode = 0;
		// Line detection
		vector<Vec2f> lines;

		for( int hl_det_cter = 5; hl_det_cter > 0; hl_det_cter--){

			HoughLines(detected_edge_up_roi, lines, 1, CV_PI/180, houghThresh, 0, 0 );
			int k = 0;
			
			for ( size_t i = 0; i < lines.size(); i++ )
			{
				rho = lines[i][0];
				theta = lines[i][1];

				if ( (theta < max_Right_Theta && theta > min_Right_Theta) || (theta < max_Left_Theta && theta > min_Left_Theta) ){
					float hl_pt_x_bottom = (rho-(ROI_Y + ROI_Height)*sin(theta))/cos(theta);
					if (hl_pt_x_bottom <= CENTER_COR_X){
						if((hl_pt_x_bottom < hl_pt_x_bottom_thrsh[0]) && (hl_pt_x_bottom > lane_width_near_min)){
							//Further improvement for average calculation for lanes
							hl_pt_x_bottom_thrsh[0] = hl_pt_x_bottom;// change from original code by comparing the rho, now it is comparing the pt_x bottom value
							ct_num_lft_det_lanes = k;
							Lpt1.x = (rho-(ROI_Height)*sin(theta))/cos(theta);// if on top = rho/cos( theta )
							// point of intersection of the line with last row
							//Point Lpt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
							Lpt2.x = hl_pt_x_bottom;
							Lpt2.y = FRAME_HEIGHT;// ROI_Y + ROI_Height
						}
					}
					else{
						if((hl_pt_x_bottom > hl_pt_x_bottom_thrsh[1]) && (hl_pt_x_bottom < lane_width_near_max)){
							//Further improvement for average calculation for lanes
							hl_pt_x_bottom_thrsh[1] = hl_pt_x_bottom;// change from original code by comparing the rho, now it is comparing the pt_x bottom value
							ct_num_rt_det_lanes = k;
							Rpt1.x = (rho-(ROI_Height)*sin(theta))/cos(theta); // if the line on top = rho/cos( theta )
							Rpt2.x = hl_pt_x_bottom;

						}
					}
					k++;
				
				}

				line(dst, Rpt1, Rpt2, Scalar(0,205,255), 1, CV_AA);
				line(dst, Lpt1, Lpt2, Scalar(0,205,255),1,CV_AA);

			}

			//Test Only//////////////////////

			line( dst, Lpt1, Lpt2, Scalar(0,0,255), 3, CV_AA);
			line( dst, Rpt1, Rpt2, Scalar(0,0,255), 3, CV_AA);
			cout << Lpt1 << " : " << Lpt2 << endl;
			cout << Rpt1 << " : " << Rpt2 << endl;


			if (ct_num_rt_det_lanes >= 0 && ct_num_lft_det_lanes >= 0){
				hl_b_vld_dbl_ln_det = 1;
			}

			if(ct_num_rt_det_lanes == -1 || ct_num_lft_det_lanes == -1){
				//key note, if not two lane is detect adjust houghline calibration
				if( houghThresh > houghthresh_adj_low_s ){
					houghThresh -= houghthresh_adj_delta_s;
				}
				else{
					houghThresh = houghthresh_adj_hi_s;
					hl_b_vld_dbl_ln_det = 0;
						
					if( ct_num_rt_det_lanes == -1 && ct_num_lft_det_lanes == -1 ){
						cout<< "No Lane Detected" << endl;
					}
					else{
						cout<<" one lane detection mode" << endl;
					}
					break;
				}
			}
			

		}



        if(ct_num_lft_det_lanes == -1 && ct_num_rt_det_lanes == -1)    // No lane Mark detected ,laneMode=0
		{
			lane_mode = 0;
		}

        if(ct_num_lft_det_lanes == -1 && ct_num_rt_det_lanes >= 0)    // Only Right lane was dectected, laneMode=1
        {
			lane_mode=1;
		}

        if(ct_num_rt_det_lanes == -1 && ct_num_lft_det_lanes >= 0)     // Only Left lane was dectected ,laneMode=2
        {
				lane_mode=2;
		}

        if(ct_num_lft_det_lanes >= 0 && ct_num_rt_det_lanes >= 0)      // Both Right and Left lane were dectected ,laneMode=3
        {
			lane_mode=3;
		}

		switch(lane_mode)
		{
			case 0:
			cout<<"No lane mark detected"<<endl;
			lane_enum_ROI1 = 0;
			Lane_Mod_Gn = 0.95;
			LaneFlag1 = 1;
			break;

			case 1:
			cout<<"Only Right lane was dectected"<<endl;
			//if(Lpt1kf.x<5)
			//{laneWFR=0;}
			Midpt1.x = (Rpt1.x-lane_width_far_rt);
			Midpt2.x = (Rpt2.x-lane_width_near);
			Midpt2.y = FRAME_HEIGHT;// ROI_Y + ROI_Height
			lane_enum_ROI1 = 1;
			Lane_Mod_Gn = 0.98;
			LaneFlag1 = 0;
			break;

			case 2:
			cout<<"Only Left lane was dectected"<<endl;
			//if(Lpt1kf.x>315)
			// {laneWFL=0;}
			Midpt1.x = (Lpt1.x+lane_width_far_lft);
			Midpt2.x = (Lpt2.x+lane_width_near);
			Midpt2.y = FRAME_HEIGHT; // ROI_Y + ROI_Height
			lane_enum_ROI1 = 2;
			Lane_Mod_Gn = 0.98;
			LaneFlag1 = 0;
			break;

			case 3:
			cout<<"Right and Left lane were dectected"<<endl;
			
			if(abs(Lpt2.x-Rpt2.x) < del_min_dist_sgl_ln_s )
			{
					if(Lpt1.x < Lpt2.x)// if top point is on the left side of the bottom point
					{
						Midpt1.x=(Rpt1.x - lane_width_far_rt);
						Midpt2.x=(Rpt2.x - lane_width_near);
						lane_enum_ROI1 = 1;
					}
					else
					{
						Midpt1.x=(Lpt1.x + lane_width_far_rt);
						Midpt2.x=(Lpt2.x+lane_width_near);
						lane_enum_ROI1 = 2;
					}
			}
			else
			{
				Midpt1.x=(Lpt1.x+Rpt1.x)/2;
				Midpt2.x=(Lpt2.x+Rpt2.x)/2;
				lane_enum_ROI1 = 0;
			}

			Midpt2.y = FRAME_HEIGHT; // ROI_Y + ROI_Height

			cout << Midpt1 << " ; " << Midpt2 << endl;
			Lane_Mod_Gn = 1.0;
			LaneFlag1 = 0;
			break;

		}
		line( dst, Midpt1, Midpt2, Scalar(255), 2);	

		lane_reg_flt_x = Midpt1.x;
		lane_assignment_midpt2 = Midpt2.x;


		//Saturation
		if (lane_reg_flt_x > x_lane_cross_max){
		lane_reg_flt_x = x_lane_cross_max;
		}
		else if(lane_reg_flt_x > x_lane_cross_min){
		// do nothing
			;
		}
		else{
		lane_reg_flt_x = x_lane_cross_min;
		}



	if (Lpt1.x > 0)
	{
		lane_width_near_min_roi2= Lpt1.x; 
	}
	else{
		lane_width_near_min_roi2 = lane_width_near_min;
	}
	
	if (Rpt1.x > 0){

		lane_width_near_max_roi2 = Rpt1.x;	
	}
	else{
		lane_width_near_max_roi2 = lane_width_near_max;
	}

	
	Old_lt_pt = Lpt1;
	Old_rt_pt = Rpt1;
	bitwise_and(detected_edges,mask_2,detected_edge_low_roi);
imshow(debug3, detected_edge_low_roi);

	/*The whole block wrapped by {} is designed for ROI2 lane detection and center return - Yue Sun Oct 8 2017*/
	
		//Used for houghline////////////////////////////////////////////////
		//Record Left/Right theta
		Left_Lane_Theta = 0;
		Right_Lane_Theta = 0;
		// Log old theta for left & right - start from 90 degree: Right Lane search largest theta value, Left Lane search smallest theta value
		Old_Left_Theta = 90;
		Old_Right_Theta = 90;

		// Log Right & Left Rho - Default value determined by frame size
		//e.g. 600 x 400, default Left Rho is 1 and Right Rho will be 600
		// Left Rho default value is
		Left_Lane_Rho = 1;
		Right_Lane_Rho = 600;
		//////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////

		//2017 - 01 - 03 Reference to Hongbo's code
		ct_num_lft_det_lanes = -1;
		ct_num_rt_det_lanes = -1;

		Rpt1.x = 0;
		Rpt1.y = ROI_2_Height;
        	Lpt1.x = 0;
		Lpt1.y = ROI_2_Height;
       		Rpt2.x = 0;
		Rpt2.y = ROI_Height;
		Lpt2.x = 0;
		Lpt2.y = ROI_Height;
	//	Midpt1.x = CENTER_COR_X;
		Midpt1.y = ROI_2_Height;
        	Midpt2.x = Midpt1.x;
		Midpt2.y = ROI_Height;

		houghThresh = 45;
		houghthresh_adj_hi_s = houghThresh;

		hl_pt_x_bottom_thrsh[0] = FRAME_WIDTH;
		hl_pt_x_bottom_thrsh[1] = 0;

		for( int hl_det_cter = 5; hl_det_cter > 0; hl_det_cter--){

			HoughLines(detected_edge_low_roi, lines, 1, CV_PI/180, houghThresh, 0, 0 );
			int k = 0;

			
			for ( size_t i = 0; i < lines.size(); i++ )
			{
				rho = lines[i][0];
				theta = lines[i][1];

				if ( (theta < max_Right_Theta && theta > min_Right_Theta) || (theta < max_Left_Theta && theta > min_Left_Theta) ){
					float hl_pt_x_bottom = (rho-(ROI_Height)*sin(theta))/cos(theta);
					if (hl_pt_x_bottom <= CENTER_COR_X){
						if((hl_pt_x_bottom < hl_pt_x_bottom_thrsh[0]) && (hl_pt_x_bottom >= lane_width_near_min_roi2)){
							//Further improvement for average calculation for lanes
							hl_pt_x_bottom_thrsh[0] = hl_pt_x_bottom;// change from original code by comparing the rho, now it is comparing the pt_x bottom value
							ct_num_lft_det_lanes = k;
							Lpt1.x = (rho-(ROI_2_Height)*sin(theta))/cos(theta);// if on top = rho/cos( theta )
							// point of intersection of the line with last row
							//Point Lpt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
							if (abs(Old_lt_pt.x - hl_pt_x_bottom) < 10 || Old_lt_pt.x < 10 )
							{
								Lpt2.x = hl_pt_x_bottom;
							}
							else
							{
								Lpt2.x = Old_lt_pt.x;
							}
							Lpt2.y = ROI_Height;// ROI_Y + ROI_Height
						}
					}
					else{
						if((hl_pt_x_bottom > hl_pt_x_bottom_thrsh[1]) && (hl_pt_x_bottom <= lane_width_near_max_roi2)){
							//Further improvement for average calculation for lanes
							hl_pt_x_bottom_thrsh[1] = hl_pt_x_bottom;// change from original code by comparing the rho, now it is comparing the pt_x bottom value
							ct_num_rt_det_lanes = k;
							Rpt1.x = (rho-(ROI_2_Height)*sin(theta))/cos(theta); // if the line on top = rho/cos( theta )
							if (abs(Old_rt_pt.x - hl_pt_x_bottom) < 10|| Old_rt_pt.x < 10  )
							{
								Rpt2.x = hl_pt_x_bottom;
							}
							else
							{
								Rpt2.x = Old_rt_pt.x;
							}

						}
					}
					k++;
				
				}

				line(dst, Rpt1, Rpt2, Scalar(0,205,255), 1, CV_AA);
				line(dst, Lpt1, Lpt2, Scalar(0,205,255),1,CV_AA);

			}

			//Test Only//////////////////////
			//red line
			line( dst, Lpt1, Lpt2, Scalar(0,0,255), 3, CV_AA);
			line( dst, Rpt1, Rpt2, Scalar(0,0,255), 3, CV_AA);
			cout << Lpt1 << " : " << Lpt2 << endl;
			cout << Rpt1 << " : " << Rpt2 << endl;


			if (ct_num_rt_det_lanes >= 0 && ct_num_lft_det_lanes >= 0){
				hl_b_vld_dbl_ln_det = 1;
				houghThresh = houghthresh_adj_hi_s;
				cout << "ROI2 double lane detected" << endl;
				break;
			}

			if(ct_num_rt_det_lanes == -1 || ct_num_lft_det_lanes == -1){
				//key note, if not two lane is detect adjust houghline calibration
				if( houghThresh > houghthresh_adj_low_s ){
					houghThresh -= houghthresh_adj_delta_s;
					break;
					//yue sun 3/27/2019 quit loop for debugging
				}
				else{
					houghThresh = houghthresh_adj_hi_s;
					hl_b_vld_dbl_ln_det = 0;
						
					if( ct_num_rt_det_lanes == -1 && ct_num_lft_det_lanes == -1 ){
						cout<< "ROI2 No Lane Detected" << endl;

					}
					else{
						cout<<" ROI2 one lane detection mode" << endl;
						cout<<ct_num_rt_det_lanes<<" "<<ct_num_lft_det_lanes<<endl;
					}
					break;
				}
			}
			

		}

	//	lane_mode = 0;

        //if(ct_num_lft_det_lanes == -1 && ct_num_rt_det_lanes == -1)    // No lane Mark detected ,laneMode=0
		//{
		//	lane_mode = 0;
		//}

	//Yue Sun 2019-03-27 remove the && condition of ROI1	
	lane_mode =0;
	cout<<ct_num_rt_det_lanes<<" "<<ct_num_lft_det_lanes<<endl;
        if(ct_num_lft_det_lanes == -1 && ct_num_rt_det_lanes >= 0 && lane_enum_ROI1 != 2 && lane_reg_flt_x <= lane_assignment_midpt2)    // Only Right lane was dectected and ROI1 is not left and ROI1 center is not left, laneMode=1
        {
			lane_mode=1;
	}
	

	//else{lane_mode = 0;}
	cout<<lane_mode;

        if(ct_num_rt_det_lanes == -1 && ct_num_lft_det_lanes >= 0 && lane_enum_ROI1 !=1 && lane_reg_flt_x >= lane_assignment_midpt2)     // Only Left lane was dectected and ROI1 is not right ,laneMode=2
        {
			lane_mode=2;	
	}
        

	//else{lane_mode = 0;}


        if(ct_num_lft_det_lanes >= 0 && ct_num_rt_det_lanes >= 0)      // Both Right and Left lane were dectected ,laneMode=3
        {
			lane_mode=3;
	}

	if(ct_num_lft_det_lanes == -1 && ct_num_rt_det_lanes == -1) {lane_mode = 0;}

	cout<<lane_mode;
		switch(lane_mode)
		{
			case 0:
			cout<<"ROI2 No lane mark detected"<<endl;
			// not recalculating the Midpt1.x for ROI2
			// using the same from ROI1...basically moving the point up
			Lane_Mod_Gn *= 0.95;
			LaneFlag2 = 1;
			break;

			case 1:
			cout<<"ROI2 Only Right lane was dectected"<<endl;
			//if(Lpt1kf.x<5)
			//{laneWFR=0;}
			Midpt1.x = (Rpt1.x-lane_width_far_rt_ROI2);
			//Midpt2.x = (Rpt2.x-lane_width_near);
			//Midpt2.y = ROI_Height;// ROI_Y + ROI_Height
			Lane_Mod_Gn *= 0.98;
			LaneFlag2 = 0;
			break;

			case 2:
			cout<<"ROI2 Only Left lane was dectected"<<endl;
			//if(Lpt1kf.x>315)
			// {laneWFL=0;}
			Midpt1.x = (Lpt1.x+lane_width_far_lft_ROI2);
		//	Midpt2.x = (Lpt2.x+lane_width_near);
		//	Midpt2.y = ROI_Height; // ROI_Y + ROI_Height
			Lane_Mod_Gn *=0.98;
			LaneFlag2 = 0;
			break;

			case 3:
			cout<<"ROI2 Right and Left lane were dectected"<<endl;
			
			if(abs(Lpt2.x-Rpt2.x) < del_min_dist_sgl_ln_ROI2_s )
			{
					if(Lpt1.x < Lpt2.x && lane_enum_ROI1 != 2)// if top point is on the left side of the bottom point
					{
						Midpt1.x=(Rpt1.x - lane_width_far_rt_ROI2);
						//Midpt2.x=(Rpt2.x - lane_width_near);
					}
					else if (lane_enum_ROI1 == 2)
					{
						Midpt1.x=(Lpt1.x + lane_width_far_lft_ROI2);
						//Midpt2.x=(Lpt2.x+lane_width_near);
					}
					else
					{// Do nothing
					}
			}
			else
			{
				Midpt1.x=(Lpt1.x+Rpt1.x)/2;
				//Midpt2.x=(Lpt2.x+Rpt2.x)/2;
			}

			Midpt2.y = ROI_Height; // ROI_Y + ROI_Height

			cout << Midpt1 << " ; " << Midpt2 << endl;
			LaneFlag2 = 0;
			break;
			default: break;

		}
		line( dst, Midpt1, Midpt2, Scalar(255), 2);	

		lane_reg_flt_x2 = Midpt1.x;


		//Saturation
		if (lane_reg_flt_x2 > x_lane_cross_max){
		lane_reg_flt_x2 = x_lane_cross_max;
		}
		else if(lane_reg_flt_x2 > x_lane_cross_min){
		// do nothing
			;
		}
		else{
		lane_reg_flt_x2 = x_lane_cross_min;
		}



		if ((Old_lt_pt.x == 0 && Old_rt_pt.x == 0) && (Lpt1.x == 0 && Rpt1.x == 0)){
			lane_reg_flt_x = lane_reg_flt_x_old;
			lane_reg_flt_x2 = lane_reg_flt_x2_old;	

		}
		else{
			lane_reg_flt_x_old = lane_reg_flt_x;
			lane_reg_flt_x2_old = lane_reg_flt_x2;
		}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////END OF HOUGLINE OLD VERSION////////////////////////////////////////////////////////////////////////////        //TEST MODE/////////////////////////////////////////////////////////////////////////////////////////
	//2016-11-08 F. Dang Debug mode enable
	if ( lane_reg_b_debug_md_enbl_s == DEBUG_MODE ){
		//Monitor only//////////////////////////////////////////////////////////////////////////////
		//src.copyTo( dst, detected_edges);
		imshow( window_name_line, dst );
		imshow(window_name, detected_edges);
	}
}
void Lane_Reg_Init(int, void*){
	//initialization for min theta
	min_Left_Theta = minLeftThetaDegrees/180.0 * CV_PI;
	//cout << "New minLeftTheta: " << min_Left_Theta << " from " << minLeftThetaDegrees << " deg" << endl;
	max_Left_Theta = maxLeftThetaDegrees/180.0 * CV_PI;
	//cout << "New maxLeftTheta: " << max_Left_Theta << " from " << maxLeftThetaDegrees << " deg" << endl;
	min_Right_Theta = minRightThetaDegrees/180.0 * CV_PI;
	//cout << "New minRightTheta: " << min_Right_Theta << " from " << minRightThetaDegrees << " deg" << endl;
	max_Right_Theta = maxRightThetaDegrees/180.0 * CV_PI;
	//cout << "New maxRightTheta: " << max_Right_Theta << " from " << maxRightThetaDegrees << " deg" << endl;

	//initalization for section threshold index = MAX_SECT_NUM+1
	for (int sect_i = MIN_SECT_NUM; sect_i <= MAX_SECT_NUM; sect_i ++){
		//section threshold
		phl_thrsh_y_sect[sect_i] = SECT_HEIGHT*sect_i;

	}

	//initialize for section index aprameter
	for (int sect_i = MIN_SECT_NUM; sect_i < MAX_SECT_NUM; sect_i ++){

		//section reference line in y direction
		phl_ln_ref_y[sect_i] = ( phl_thrsh_y_sect[sect_i + 1] + phl_thrsh_y_sect[sect_i] )/2;
		//calculated center point
		phl_pt_x_sect_ct[sect_i] = CENTER_COR_X;// when double lane detected mode
		phl_pt_x_sect_ct_lft_est[sect_i] = CENTER_COR_X;// when single left lane detected mode
		phl_pt_x_sect_ct_rt_est[sect_i] = CENTER_COR_X;// when single left lane detected mode

		phl_pt_intc_x_2_ct_del_lrn[sect_i] = 0;// TO DO need to have a default lane with and calculate for each section

		//phl_coef_sect_trans[sect_i] = (phl_coef_sect_trans_s)^(MAX_SECT_NUM - sect_i - 1);

		for (int loop_j = 0; loop_j < MAX_LOOP_NUM; loop_j++){

			phl_pt_x_ct_loop[sect_i][loop_j] = CENTER_COR_X;

		}

		lane_reg_buf[0] = CENTER_COR_X;

		lane_reg_flt_x = CENTER_COR_X;

		lane_reg_buf_index = 0;


	}

	/*
	//initialize from bottom section for transition coef
	for (int sect_i = (MAX_SECT_NUM-1); sect_i >= MIN_SECT_NUM; sect_i --){

		if (sect_i = (MAX_SECT_NUM-1)){
			phl_coef_sect_trans[sect_i] = 1;

		}
		else{
			phl_coef_sect_trans[sect_i] = phl_coef_sect_trans[sect_i + 1]*phl_coef_sect_trans_s;
		}

	}
	*/

	Mat tmp;
	CannyThreshold(0, 0, tmp);
}


//post processing calculated center point data
void lane_reg_ct_x_proc(int, void*){

	/////2016-09-10 F. Dang: if Double lane detect mode Set Flag for valid result
		if ( phl_b_vld_ln_det[RIGHT_LANE] == 1 && phl_b_vld_ln_det[LEFT_LANE] == 1 ) {
				//start check clostest section 

			
			for (int sect_i = (MAX_SECT_NUM-1); sect_i >= MIN_SECT_NUM; sect_i --){

				if( phl_b_vld_ln_det_sec[sect_i][LEFT_LANE] == 1 && phl_b_vld_ln_det_sec[sect_i][RIGHT_LANE] == 1 ){
					lane_reg_buf[lane_reg_buf_index] = phl_pt_x_sect_ct[sect_i]; 
					//lane_reg_buf[lane_reg_buf_index] = CENTER_COR_X + (phl_pt_x_sect_ct[sect_i] - CENTER_COR_X)*phl_coef_sect_trans[sect_i];
					lane_reg_flt_x = lane_reg_buf[lane_reg_buf_index];
					break;
				}
				else{
					//TBD coef need to be considered for each section
				}


			}
			
			/*
			//lane_reg_buf_index already been processed at previous detection step
			lane_reg_buf[lane_reg_buf_index] = phl_pt_x_sect_ct[MAX_SECT_NUM]; 	
			lane_reg_flt_x = lane_reg_buf[lane_reg_buf_index];
			*/
			//////////////////////////////////////////////////////////////////////////////////////////////////////////
			//TEST MODE/////////////////////////////////////////////////////////////////////////////////////////
			//2016-11-08 F. Dang Debug mode enable
			if ( lane_reg_b_debug_md_enbl_s == DEBUG_MODE ){
				cout << "Filtered_X_DL_V: " << lane_reg_flt_x << endl;
			}
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////////////////////////////////////
		}

		//Single line detection mode - Left
		else if( phl_b_vld_ln_det[RIGHT_LANE] != 1 && phl_b_vld_ln_det[LEFT_LANE] == 1 ){	

			if(lane_reg_buf_index == 0){

				float pt_left_intsct_del_tmp = 	phl_pt_x_sect_lft_line[lane_reg_buf_index] - phl_pt_x_sect_lft_line[BUF_SIZE - 1];
				//need to update 
				lane_reg_buf[lane_reg_buf_index] = lane_reg_buf[BUF_SIZE - 1] - pt_left_intsct_del_tmp;
				lane_reg_flt_x = lane_reg_buf[lane_reg_buf_index];

			}
			else{

				float pt_left_intsct_del_tmp = 	phl_pt_x_sect_lft_line[lane_reg_buf_index] - phl_pt_x_sect_lft_line[lane_reg_buf_index-1];
				lane_reg_buf[lane_reg_buf_index] = lane_reg_buf[lane_reg_buf_index - 1] - pt_left_intsct_del_tmp;
				lane_reg_flt_x = lane_reg_buf[lane_reg_buf_index];

			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////////
			//TEST MODE/////////////////////////////////////////////////////////////////////////////////////////
			//2016-11-08 F. Dang Debug mode enable
			if ( lane_reg_b_debug_md_enbl_s == DEBUG_MODE ){
				cout << "Filtered_X_SNG_LEFT_V: " << lane_reg_flt_x << endl;
			}
		}

		//Single line detection mode - Left
		else if( phl_b_vld_ln_det[RIGHT_LANE] == 1 && phl_b_vld_ln_det[LEFT_LANE] != 1 ){
				

			if(lane_reg_buf_index == 0){
				float pt_right_intsct_del_tmp = phl_pt_x_sect_rt_line[lane_reg_buf_index] - phl_pt_x_sect_rt_line[BUF_SIZE - 1];
				//need to update 
				lane_reg_buf[lane_reg_buf_index] = lane_reg_buf[BUF_SIZE - 1] + pt_right_intsct_del_tmp;
				lane_reg_flt_x = lane_reg_buf[lane_reg_buf_index];

			}
			else{
				float pt_right_intsct_del_tmp = phl_pt_x_sect_rt_line[lane_reg_buf_index] - phl_pt_x_sect_rt_line[lane_reg_buf_index - 1];

				lane_reg_buf[lane_reg_buf_index] = lane_reg_buf[lane_reg_buf_index - 1] + pt_right_intsct_del_tmp;
				lane_reg_flt_x = lane_reg_buf[lane_reg_buf_index];

			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////////
			//TEST MODE/////////////////////////////////////////////////////////////////////////////////////////
			//2016-11-08 F. Dang Debug mode enable
			if ( lane_reg_b_debug_md_enbl_s == DEBUG_MODE ){
				cout << "Filtered_X_SNG_RIGHT_V: " << lane_reg_flt_x << endl;
			}
		}


		else{

			if(lane_reg_buf_index == 0){
				lane_reg_buf[lane_reg_buf_index] = lane_reg_buf[BUF_SIZE - 1];
				lane_reg_flt_x = lane_reg_buf[lane_reg_buf_index];
			}
			else{

				lane_reg_buf[lane_reg_buf_index] = lane_reg_buf[lane_reg_buf_index-1];
				lane_reg_flt_x = lane_reg_buf[lane_reg_buf_index];
			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////////
			//TEST MODE/////////////////////////////////////////////////////////////////////////////////////////
			//2016-11-08 F. Dang Debug mode enable
			if ( lane_reg_b_debug_md_enbl_s == DEBUG_MODE ){
			
				cout << "Filtered_X_Not_Valid: " << lane_reg_flt_x << endl;
			}
			
		}
		
		//Saturation for Coordinate X before deliver to decision making
		if (lane_reg_flt_x > x_lane_cross_max){
			lane_reg_flt_x = x_lane_cross_max;
		}
		else if(lane_reg_flt_x > x_lane_cross_min){
			lane_reg_flt_x = lane_reg_flt_x;
		}
		else{
			lane_reg_flt_x = x_lane_cross_min;
		}
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////
		//TEST MODE/////////////////////////////////////////////////////////////////////////////////////////
		//2016-11-08 F. Dang Debug mode enable
		if ( lane_reg_b_debug_md_enbl_s == DEBUG_MODE ){
		
			cout << "lane_reg_flt_x: " << lane_reg_flt_x << endl;
		}

}
