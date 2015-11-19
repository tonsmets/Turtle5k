#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

ros::Publisher pFramePub;

using namespace cv;

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k-camera");
	ros::NodeHandle pHandle;
	ros::Rate pRate(24);

	Mat image;
	image = imread("binary_includes/Turtle-5K.png", CV_LOAD_IMAGE_COLOR);   // Read the file

	if(! image.data )                              // Check for invalid input
	{
		std::cout <<  "Could not open or find the image" << std::endl ;
		return -1;
	}

	namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    	imshow( "Display window", image );  
	
	pFramePub = pHandle.advertise<std_msgs::String>("/t5k/frame", 1000);
	while(ros::ok()) {
		std_msgs::String pMessage;
		pMessage.data = "frame:{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}";
		pFramePub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}
	
	return 0;
}
