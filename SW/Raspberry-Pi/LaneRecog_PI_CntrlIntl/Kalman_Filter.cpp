//#include "tracking.hpp"

#include "lane_recog.h"
#include "Kalman_filter.h"


//Constants///////////////////////////////////////////////////////////////////////
//int const MAX_SECT_NUM = 10;// refer to lane reg	

////calibration///////////////////////////////////////////////////////////////////
/*
float phl_coef_sect_trans[MAX_SECT_NUM];//section transition coef


void Kalman_Filter(int sect_i, void*){
	
	//setIdentity(phl_coef_sect[MAX_SECT_NUM], Scalar::all(CENTER_COR_X));

	KalmanFilter KF( 3, 1, 0);
	
	KF.transitionMatrix = (Mat_<float>(3, 3) << 1,phl_coef_sect_trans[sect_i],0,   0,1,phl_coef_sect_trans[sect_i],  0,0,1);

	Mat_<float> measurement(1,1); measurement.setTo(Scalar(CENTER_COR_X));
 
	// init...
	KF.statePre.at<float>(0) = CENTER_COR_X;
	KF.statePre.at<float>(1) = 0;
	KF.statePre.at<float>(2) = 0;
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(.1));

	// First predict, to update the internal statePre variable
	Mat prediction = KF.predict();
	//Point predictPt(prediction.at<float>(0),phl_ln_ref_y[sect_i]);
             
	// Get measurement
	measurement(0) = phl_pt_x_sect_ct[sect_i];
             
	//Point measPt(measurement(0),phl_ln_ref_y[sect_i]);
 
	// The "correct" phase that is going to use the predicted value and our measurement
	Mat estimated = KF.correct(measurement);
	//Point statePt(estimated.at<float>(0),phl_ln_ref_y[sect_i]);



}

*/