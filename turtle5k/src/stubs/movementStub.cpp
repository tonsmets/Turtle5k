#include "ros/ros.h"
#include "std_msgs/String.h"

#include "turtle5k/MovementMessage.h"

#include "../hardware/headers/MovementTestStub.h"

ros::Publisher pVelocityPub;
ros::Publisher pPIDRegulationPub;
ros::Subscriber pWheelControlCmdSub;
ros::Subscriber pEncoderDataSub;

MovementTestStub movement;

void wheelcontrolcommandsCallback(const turtle5k::MovementMessage movementMsg)
{
	ROS_INFO("[WHEELCONTROL] Got %f %f, From: COMMANDS", movementMsg.pSpeed, movementMsg.pRadians);

	movement.setAngle(movementMsg.pRadians);
	ROS_INFO("setAngle: [%f]", movementMsg.pRadians);

	movement.setSpeed(movementMsg.pSpeed);
	ROS_INFO("setSpeed: [%f]", movementMsg.pSpeed);
}

void encoderDataCallback(const std_msgs::String::ConstPtr& aMessage)
{
	ROS_INFO("[WHEELCONTROL] Got %s, From: WHEELDRIVER", aMessage->data.c_str());
	//std_msgs::String pMessage = getVelocity();
	//pVelocityPub.publish(pMessage);
}

int main(int argc, char** argv) {

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	ros::init(argc, argv, "t5k-wheelcontrol");
	ros::NodeHandle pHandle;
	
	pVelocityPub = pHandle.advertise<std_msgs::String>("/t5k/velocity", 1000);
	pPIDRegulationPub = pHandle.advertise<std_msgs::String>("/t5k/pidregulation", 1000);
	pWheelControlCmdSub = pHandle.subscribe("/t5k/wheelcontrolcommands", 1000, wheelcontrolcommandsCallback);
	pEncoderDataSub = pHandle.subscribe("/t5k/encoderdata", 1000, encoderDataCallback);
	
	ros::Rate loop_rate(10);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
		
	return 0;
}
