// OpenCV basic blob detection
// Last modified by Ben Wang, 11/18/2015

// Color for command Go learned: (53, 141, 63), (83, 248, 166)
// Color for command Stop learned: (0, 118, 68), (24, 248, 142)


#include "Sign_Reg.h"



/*
int filtLowHGO = 0;
int filtHighHGO = 120;

int filtLowSGO = 0;
int filtHighSGO = 100;

int filtLowVGO = 0;
int filtHighVGO = 50;

int filtLowHSTOP = 0;
int filtHighHSTOP = 180;

int filtLowSSTOP = 0;
int filtHighSSTOP = 255;

int filtLowVSTOP = 0;
int filtHighVSTOP = 255;
*/


/*
green
HSV lower: [60 - sensitivity, 100, 100]
HSV higher: [60 + sensitivity, 255, 255]
// sensitivity is a int, typically set to 15 - 20 
If image is quite dark, you need to use a lower bound for V. With these values:
[60 - sensitivity, 100, 50]  // lower bound
[60 + sensitivity, 255, 255] // upper bound

lower_red_hue_range // work for some printed red colors
HSV lower:(0, 100, 100) 
HSV higher:(10, 255, 255)

upper_red_hue_range // work for red LED
HSV lower: (160, 100, 100)
HSV higher:(179, 255, 255)
*/

int filtLowHGO = 40;
int filtHighHGO = 100;

int filtLowSGO = 100;
int filtHighSGO = 255;

int filtLowVGO = 50;
int filtHighVGO = 255;

int filtLowHSTOP = 165; //was 160
int filtHighHSTOP = 173; // was 179

int filtLowSSTOP = 100;
int filtHighSSTOP = 255;

int filtLowVSTOP = 100;
int filtHighVSTOP = 255;

int filtLowH[2] = {filtLowHGO,filtLowHSTOP};
int filtHighH[2] = {filtHighHGO,filtHighHSTOP};

int filtLowS[2] = {filtLowSGO,filtLowSSTOP};
int filtHighS[2] = {filtHighSGO,filtHighSSTOP};

int filtLowV[2] = {filtLowVGO,filtLowVSTOP};
int filtHighV[2] = {filtHighVGO,filtHighVSTOP};
int minArea = 20000;//was 10000
int MinArea = 20000;
int SignFlag= 0; // 0 for Go , 1 for Stop , and 5 for no traffic light detected
int SignFlagChanged=0;
Moments frame_moments;
int currFlagfNum=0;

Mat src_sign;

const int ROI_SIGN = 0;

const int ROI_SIGN_HEIGHT = 150;

void Sign_Reg_Init(int, void*){
 filtLowH[0] = filtLowHGO;
 filtLowH[1] = filtLowHSTOP;
 filtHighH[0] = filtHighHGO;
 filtHighH[1] = filtHighHSTOP;
 filtLowS[0] = filtLowSGO;
 filtLowS[1] = filtLowSSTOP;

 filtHighS[0] = filtHighSGO;
 filtHighS[1] = filtHighSSTOP;

 filtLowV[0] = filtLowVGO;
 filtLowV[1] = filtLowVSTOP;

 filtHighV[0] = filtHighVGO;
 filtHighV[1] = filtHighVSTOP;
 minArea = MinArea;
}


const int FILT_HSV_MAX = 255;
const int FILT_MIN_AREA_MAX = 1000000;

int blob_currX;
int blob_currY;
// Structs for learning color
string commandNames[] = {"Go", "Stop"};  // Command name strings
const int NUM_COMMANDS = 2;                         // Number of commands
enum CommandType{GO, STOP};                         // Enum of commands

class Command{
public:
    // Color limits
    Scalar lowerColorLimit;
    Scalar upperColorLimit;

    // Coordinates
    int posX;
    int posY;
	int momentArea;

    // Area
    int filteredArea;

    Command(){
        lowerColorLimit = Scalar(Scalar::all(0));
        upperColorLimit = Scalar(Scalar::all(255));
    }
private:

};

Command commands[NUM_COMMANDS]; // Array of Comman structs

// Global current color detected
enum CommandType blob_currCommand;


// Mats

Mat frame_filtered_go;
Mat frame_filtered_stop;

// Functions
//int learnColor(enum CommandType cmd);
//inline void captureAndFilter();
//inline void displayImages();

//Mat currFrame = Mat(frame_filtered.size(), frame_filtered.type());
Mat currFrameGo = Mat(frame_filtered_go.size(), frame_filtered_go.type());
Mat currFrameStop = Mat(frame_filtered_stop.size(), frame_filtered_stop.type());


enum CommandType currMaxCommand = (CommandType) 0;
int currMaxArea = 1;//was -1
bool areaFound = false;



Mat im_with_keypointsGo;
Mat im_with_keypointsStop;

void SignReg(Mat frame_hsv,Mat frame){


    // Stage 1: Calibrate color and size
    // TODO: Variable number of colors
    // Display screen for "Set color GO"
    //learnColor(GO);
    // Display screen for "Set color STOP"
    //learnColor(STOP);

    // Main loop
    
    // Capture frame
    //cap >> frame;
    //cvtColor(frame, frame_hsv, CV_BGR2HSV);

    // Reset frame_filtered to black
    frame_filtered_go = Mat::zeros(frame.size(),  CV_8U);
    frame_filtered_stop = Mat::zeros(frame.size(),  CV_8U);

    // Tracker for largest are detected
    currMaxCommand = (CommandType) 0;
    currMaxArea = 1;//was -1
	areaFound = false;
	SignFlagChanged=0;


	
	// Setup SimpleBlobDetector parameters.
	//SimpleBlobDetector::Params  params;
	//SimpleBlobDetector detector; 
	
	// Storage for blobs
	//std::vector<KeyPoint> keypoints;

	// Filter by white color
	//params.filterByColor = true;
	//params.blobColor = 255;


	// Change thresholds
	//params.minThreshold = 0;
	//params.maxThreshold = 300;
 
	// Filter by Area.
	//params.filterByArea = true;
	//params.minArea = minArea;

	// Filter by Circularity
	//params.filterByCircularity = true;
	//params.minCircularity = 0.1;
 
	// Filter by Convexity
	//params.filterByConvexity = true;
	//params.minConvexity = 0.87;
 
	// Filter by Inertia
	//params.filterByInertia = true;
	//params.minInertiaRatio = 0.01;
	
	// Set up detector with params
	//SimpleBlobDetector detector(params); //OpenCv2
	//Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);//OpenCV3






    // Iterate over each filter, add to output
    for (int fNum = 0; fNum < NUM_COMMANDS; fNum++){
		
		commands[fNum].lowerColorLimit = Scalar(filtLowH[fNum], filtLowS[fNum], filtLowV[fNum]);
        commands[fNum].upperColorLimit = Scalar(filtHighH[fNum], filtHighS[fNum], filtHighV[fNum]);

		currMaxCommand = (CommandType) fNum;

		if (fNum == 0){
			// Perform filter
			inRange(frame_hsv, commands[fNum].lowerColorLimit, 
				commands[fNum].upperColorLimit, currFrameGo);
			
			  //morphological opening (removes small objects from the foreground)
			  //erode(currFrameGo, currFrameGo, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
			  //dilate( currFrameGo, currFrameGo, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 

			   //morphological closing (removes small holes from the foreground)
			  //dilate( currFrameGo, currFrameGo, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 
			  //erode(currFrameGo, currFrameGo, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

			
			//detector->detect( frame, keypoints);//( currFrameGo, keypoints)
/*			if (detector.empty() != 0)
			{
				cout << "Blob detected" << endl;
			}

			if (keypoints.size() != 0)
			{
				cout << "Blob detected k" << endl;
			}

			cout << "No Blob" << keypoints.size() << endl;
			cout << "No Blob" << detector.empty() << endl;*/

			// Show blobs
			//drawKeypoints( frame, keypoints, im_with_keypointsGo, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
			//imshow("keypoints", im_with_keypointsGo );

			// Get moments
			frame_moments = moments(currFrameGo);
		}
		else {
			// Perform filter
			inRange(frame_hsv, commands[fNum].lowerColorLimit, 
				commands[fNum].upperColorLimit, currFrameStop);

			   //morphological closing (removes small holes from the foreground)
/*			dilate( currFrameStop, currFrameStop, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 
			erode(currFrameStop, currFrameStop, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


			detector->detect( currFrameStop, keypoints);
			cout << "No Blob" << keypoints.size() << endl;
			cout << "No Blob" << detector.empty() << endl;*/
			// Show blobs
			//drawKeypoints( frame, keypoints, im_with_keypointsStop, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
			//imshow("keypointsStop", im_with_keypointsStop );

			// Get moments
			frame_moments = moments(currFrameStop);
		}

        // Store x and y 
        int momentArea = commands[fNum].momentArea = (int) frame_moments.m00;
        int momentX = commands[fNum].posX = (int) (frame_moments.m10 / momentArea);
        int momentY = commands[fNum].posY = (int) (frame_moments.m01 / momentArea);

		cout << "MomentArea" << momentArea << endl;



        // Store max X and Y if significant
        if (momentArea > minArea && momentArea > currMaxArea){
			areaFound = true;
            currMaxArea = momentArea;
            currMaxCommand = (CommandType) fNum;
			currFlagfNum = fNum;

			// TODO: Set global X and Y variables
			blob_currX = commands[currMaxCommand].posX;
			blob_currY = commands[currMaxCommand].posY;
			blob_currCommand = currMaxCommand;


			// TODO: Draw on original frame
			//circle(frame, Point(blob_currX, blob_currY), std::sqrt(currMaxArea / CV_PI) / 20, Scalar(0, 0, 255), 5);
			if (fNum == 0){
				circle(currFrameGo, Point(blob_currX, blob_currY), 10, Scalar(0,100,255), 2, CV_AA);
			}
			else {
				circle(currFrameStop, Point(blob_currX, blob_currY), 10, Scalar(0,100,255), 2, CV_AA);
			}

			int fontFace = FONT_HERSHEY_COMPLEX_SMALL;
			double fontScale = 1;
			int thickness = 2;
			if (fNum == 0){
				putText(currFrameGo, commandNames[blob_currCommand], Point(blob_currX, blob_currY), fontFace, fontScale,Scalar(0, 0, 255), thickness,8);
			}
			else {
				putText(currFrameStop, commandNames[blob_currCommand], Point(blob_currX, blob_currY), fontFace, fontScale,Scalar(0, 0, 255), thickness,8);
			}
			//putText(currFrame, commandNames[blob_currCommand], Point(blob_currX, blob_currY), fontFace, fontScale,Scalar(0, 0, 255), thickness,8);

        }

		if (areaFound == true){
			SignFlag = currFlagfNum;
			SignFlagChanged=1;
			cout << "commandNames  " << commandNames[blob_currCommand] << endl;
			//cout << "Sign Flag  " << SignFlag << endl;

			cout << "Area Found" << areaFound << endl;
		}

        // Dye filtered frame with filter color
        // Mat currFrameBGR, currFrameColor;
        // currFrame.setTo(currFrameColor, commands[fNum].lowerColorLimit);
        // cvtColor(currFrame, currFrameBGR, CV_HSV2BGR);	

		// Draw dot and circle of command color on filtered frame
		//circle(currFrame, Point(momentX, momentY), 20 /*std::sqrt(momentArea / PI)*/, /*commands[fNum].lowerColorLimit*/ Scalar(150 + 70 * fNum), 5);

        // Add filtered image
        // frame_filtered += currFrameBGR;

    }
        frame_filtered_go += currFrameGo;
		frame_filtered_stop += currFrameStop;
		if (SignFlagChanged==0){
		SignFlag = 5; //Put Sign Flag as 5 meaning that it didn't recognized a sign
		}


	/*
	// Draw position of identified color
	if (areaFound){
		// TODO: Set global X and Y variables
		blob_currX = commands[currMaxCommand].posX;
		blob_currY = commands[currMaxCommand].posY;
		blob_currCommand = currMaxCommand;


		// TODO: Draw on original frame
		//circle(frame, Point(blob_currX, blob_currY), std::sqrt(currMaxArea / CV_PI) / 20, Scalar(0, 0, 255), 5);
			
		circle(frame_filtered, Point(blob_currX, blob_currY), 10, Scalar(0,100,255), 2, CV_AA);

		int fontFace = FONT_HERSHEY_COMPLEX_SMALL;
		double fontScale = 1;
		int thickness = 2;
		putText(frame_filtered, commandNames[blob_currCommand], Point(blob_currX, blob_currY), fontFace, fontScale,Scalar(0, 0, 255), thickness,8);
	}
	*/


	// Display all frames
    //displayImages();
	imshow("Filtered Go", frame_filtered_go);
	imshow("Filtered Stop", frame_filtered_stop);
    // Exit if any key pressed
   /* int currKey = waitKey(30);
    if(currKey == KEY_ESC || currKey == 'q') break;
    */
}



/*
int learnColor(enum CommandType cmd){
    int currKey = -1;   // Key pressed while in loop

    // TODO: Display message on screen

    // TODO: loop for learning this color
    while (1){
        // Capture current frame
        captureAndFilter();

        // Display area on screen
        int fontFace = FONT_HERSHEY_COMPLEX_SMALL;
        double fontScale = 1;
        int thickness = 2;
        Point textOrg(0 frame.cols/5, frame.rows/1.2);
        stringstream ss;
        ss << "Select color and press Enter for cmd: " << commandNames[cmd];
        string displayText = ss.str();
        putText(frame, displayText, textOrg, fontFace, fontScale,
            Scalar::all(255) Scalar(0, 0, 255), thickness,8);

		// Check area
		// Get moments
        Moments frame_moments = moments(frame_filtered);

        // Store x and y 
        int momentArea = frame_moments.m00;

		stringstream ss2;
        ss2 << "Area: " << momentArea << " is enough? " << (momentArea > minArea);
        displayText = ss2.str();
        putText(frame_filtered, displayText, textOrg, fontFace, fontScale, Scalar(150), thickness,8);


        // Display images
        displayImages();


        // Grab current key
        currKey = waitKey(30);
        if(currKey == KEY_ESC || currKey == KEY_ENTER || currKey == 'q' || currKey == KEY_ENTER_WINDOWS) break;
    }

    // TODO: Save color to cmd
    if (currKey == KEY_ENTER || currKey == KEY_ENTER_WINDOWS){
        // Save color to cmd
        commands[cmd].lowerColorLimit = Scalar(filtLowH, filtLowS, filtLowV);
        commands[cmd].upperColorLimit = Scalar(filtHighH, filtHighS, filtHighV);

        // Print color learned
        cout << "Color for command " << commandNames[cmd] << " learned: (" <<
            commands[cmd].lowerColorLimit[0] << ", " << commands[cmd].lowerColorLimit[1] << ", " << 
            commands[cmd].lowerColorLimit[2] << "), (" <<
            commands[cmd].upperColorLimit[0] << ", " << commands[cmd].upperColorLimit[1] << ", " << 
            commands[cmd].upperColorLimit[2] << ")" << endl;
    }

    return 0;
}
*/

/*
inline void captureAndFilter(){
    // Capture frame
    cap >> frame;
    cvtColor(frame, frame_hsv, CV_BGR2HSV);

    // Perform filter
    inRange(frame_hsv, Scalar(filtLowH, filtLowS, filtLowV), 
        Scalar(filtHighH, filtHighS, filtHighV), frame_filtered);
}


inline void displayImages(){
    // Display image
    imshow("Raw", frame);
    imshow("HSV", frame_hsv);
    imshow("Filtered", frame_filtered);
}
*/
