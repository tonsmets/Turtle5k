#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

ros::Publisher pFramePub;

using namespace cv;

int main(int argc, char** argv) {
	ros::init(argc, argv, "t5k_camera");
	ros::NodeHandle pHandle;
	ros::Rate pRate(24);

	Mat image;

	image = Mat::zeros(Size(640,480), CV_8UC3);

	putText(image, "OpenCV Test", Point(20,30), FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,255,0));
	
	pFramePub = pHandle.advertise<std_msgs::String>("/t5k/frame", 1000);
	while(ros::ok()) {
		imshow( "Display window", image );
		waitKey(20);

		std_msgs::String pMessage;
		pMessage.data = "frame:{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}";
		pFramePub.publish(pMessage);
		ros::spinOnce();
		pRate.sleep();
	}
	
	return 0;
}
