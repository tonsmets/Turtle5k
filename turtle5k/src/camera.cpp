#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

ros::Publisher pFramePub;

using namespace cv;
using namespace std;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

const int centerX = 326;
const int centerY = 187;
const int squareSide = 30;

const string trackbarWindowName = "Trackbars";

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 10;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 2*2;
const int MAX_OBJECT_AREA = 30 * 30;

struct thresholds{
	int H_MIN;
	int H_MAX;
	int S_MIN;
	int S_MAX;
	int V_MIN;
	int V_MAX;
} ;

vector<thresholds> allThresholds;

void on_trackbar(int, void*)
{

}

void createTrackbars(){
	//create window for trackbars
	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "H_MIN");
	sprintf(TrackbarName, "H_MAX");
	sprintf(TrackbarName, "S_MIN");
	sprintf(TrackbarName, "S_MAX");
	sprintf(TrackbarName, "V_MIN");
	sprintf(TrackbarName, "V_MAX");
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
}

void threshImage(Mat image, Mat &thresholded, vector<thresholds> thr) {
	Mat tmp = image.clone();
	Mat totalOut = image.clone();
	for(int i = 0; i < thr.size(); i++) {
		thresholds tmpThresh = thr.at(i);
		Mat tmpout;
		inRange(image, Scalar(tmpThresh.H_MIN, tmpThresh.S_MIN, tmpThresh.V_MIN), 
			Scalar(tmpThresh.H_MAX, tmpThresh.S_MAX, tmpThresh.V_MAX), tmpout);
		if(i == 0) {
			totalOut = tmpout;
		}
		else {
			bitwise_or(totalOut, tmpout, totalOut);
		}
	}
	thresholded = totalOut;
}

void threshSingle(Mat image, Mat &thresholded, thresholds thr) {
		inRange(image, Scalar(thr.H_MIN, thr.S_MIN, thr.V_MIN), 
			Scalar(thr.H_MAX, thr.S_MAX, thr.V_MAX), thresholded);
}

float angleBetween(const Point &v1, const Point &v2)
{
    float len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    float len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

    float dot = v1.x * v2.x + v1.y * v2.y;

    float a = dot / (len1 * len2);

    if (a >= 1.0)
        return 0.0;
    else if (a <= -1.0)
        return M_PI;
    else
        return acos(a); // 0..PI
}

void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS){
			double angle;
			Mat globalContour;
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				globalContour = (cv::Mat)contours[index];
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					x = (int)(moment.m10 / area);
					y = (int)(moment.m01 / area);
					objectFound = true;
					refArea = area;
					angle = (0.5*atan2((2 * moment.mu11), (moment.mu20 - moment.mu02))) * (180 / M_PI);
				}
				else objectFound = false;


			}
			//let user know you found an object
			if (objectFound == true){
				//putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
				RotatedRect minRectangle = minAreaRect(globalContour);
				Point2f rect_points[4]; 

				// Get the points for
				minRectangle.points(rect_points);

				// Draw all sides of the rectangle
				for (int j = 0; j < 4; j++) {
					line(cameraFeed, rect_points[j], rect_points[(j + 1) % 4], Scalar(20,150,20), 2, 8);
				}
				
				line(cameraFeed, Point(centerX, centerY), Point(x, y), Scalar(150,150,20), 2);
				float diffAngle = angleBetween(Point(centerX, 0), Point(x,y));
				diffAngle = (diffAngle * 180) / M_PI;
				//cout << diffAngle << endl;
				int currAngle = diffAngle;
				putText(cameraFeed, ("Angle: " + to_string(currAngle)), Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0));
			}

		}
	}
}

void morphOps(Mat &thresh){
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(2, 2));
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(5, 5));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_camera");
	ros::NodeHandle pHandle;
	ros::Rate pRate(24);
	
	VideoCapture cap("/home/viki/testvideo.mp4");

	Mat image;
	Mat output;
	
	createTrackbars();
	
	thresholds ballThresh = { H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX };
	
	thresholds thresh1 = {23, 151, 136, 256, 256, 256};
	thresholds thresh2 = {0, 30, 91, 256, 161, 256};
	thresholds thresh3 = {8, 159, 201, 256, 0, 256};
	thresholds thresh4 = {0, 61, 99, 256, 144, 256};
	
	allThresholds.push_back(thresh1);
	allThresholds.push_back(thresh2);
	allThresholds.push_back(thresh3);
	//allThresholds.push_back(thresh4);
	
	pFramePub = pHandle.advertise<std_msgs::String>("/t5k/frame", 1000);
	
	int frameCount = 0;
	
	while(ros::ok()) {
		frameCount += 1;
		
		cap >> image;
		resize(image, image, Size(640, 360));
		rectangle(image, Point(centerX-squareSide/2, centerY-squareSide/2), Point(centerX+squareSide/2, centerY+squareSide/2), 20, 8);
		circle(image, Point(centerX, centerY), 267, Scalar(0, 0, 0), 175);

		if (frameCount == cap.get(CV_CAP_PROP_FRAME_COUNT)) {
			frameCount = 0;
			cap.set(CV_CAP_PROP_POS_FRAMES, 0);
		}
		
		ballThresh = { H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX };
		//threshSingle(image, output, ballThresh);
		threshImage(image, output, allThresholds);
		
		morphOps(output);
		
		line(image, Point(centerX, centerY), Point(centerX, 0), Scalar(0, 0, 255), 3);
		
		int x = 0;
		int y = 0;
		
		trackFilteredObject(x, y, output, image);
		
		//resize(output, output, Size(640, 360));
		
		
		imshow( "Display window", output );
		imshow( "Original", image);
		waitKey(20);

		std_msgs::String pMessage;
		pMessage.data = "frame:{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}";
		pFramePub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}
	
	return 0;
}
