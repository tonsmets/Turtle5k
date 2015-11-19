#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

ros::Publisher pFramePub;

#define DEBUG false

using namespace cv;

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_camera");
	ros::NodeHandle pHandle;
	ros::Rate pRate(24);

	if(DEBUG) {
		Mat image;
		image = imread("/home/viki/Turtle-5K.png", CV_LOAD_IMAGE_COLOR); 

		if(! image.data ) {
			std::cout <<  "Could not open or find the image" << std::endl ;
			return -1;
		}

		namedWindow( "Display window", WINDOW_AUTOSIZE );
	    	imshow( "Display window", image ); 

		waitKey(20); 
	}
	
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
