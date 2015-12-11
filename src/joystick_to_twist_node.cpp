/*****************************************************************************************************************************************
Function: 
This program converts the joystick message from joynode to a twist message for a turtle 5k.

Pre:


Post:
vector that gives x and y direction in m/s. and a rotation around de z-ax in rad/s

Writer 		: Vincent Bevaart
date 		: 1-12-2015
********************************************************************************************************************************************/
// includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define ROS_FRAME_RATE 					100
#define SUB_TOPIC_NAME 					"joy"
#define SUB_TOPIC_BUFFER_SIZE			1
#define PUB_TOPIC_NAME 					"motorspeed_set"
#define PUB_TOPIC_BUFFER_SIZE			1


/*****************************************************************************************************************************************
Start defining class Subscribe
********************************************************************************************************************************************/
class Subscriber
{
public:
	//axis values
	double dVelAxisX;
	double dVelAxisY;
	double dVelAxisZ;

///////////////////////////////////////////////////////////////////////////////////////////
//Function: makes publisher and subscriber and read the params for this node.
//pre: 	-
//post: 
///////////////////////////////////////////////////////////////////////////////////////////
	Subscriber();
	void sendValue();

private:
	///////////////////////////////////////////////////////////////////////////////////////////
	//Function: read joystick message.
	//pre: 	
	//post: 
	/////////////////////////////////////////////////////////////////////////////////////////// 
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  

	int iAxisX, iAxisY, iAxisZ;			//define joystick button for linear x, y and angular z
	double dScaleAxisX, dScaleAxisY, dScaleAxisZ;	
	ros::NodeHandle nh_;	
	ros::Publisher pub_VelPub;
	ros::Subscriber sub_VelSub;
	geometry_msgs::Twist msg;
  
};
/*****************************************************************************************************************************************
end of defining class Subscribe
********************************************************************************************************************************************/

/*****************************************************************************************************************************************
Start of main
********************************************************************************************************************************************/
int main(int argc, char** argv){
	
	//init
	ros::init(argc, argv, "joystick_to_twist_node");
	ROS_INFO_ONCE("ROS is initialized");
  	Subscriber sub;

  	//set frame rate for ROS while loop
	ros::Rate loop_rate(ROS_FRAME_RATE);


	while(ros::ok())
	{
		sub.sendValue();
		ROS_INFO_ONCE("first value is send");
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

/*****************************************************************************************************************************************
End of main
********************************************************************************************************************************************/

/*****************************************************************************************************************************************
Functions
********************************************************************************************************************************************/

///////////////////////////////////////////////////////////////////////////////////////////
//Function: makes publisher and subscriber and read the params for this node.
//pre: 	-
//post: 
///////////////////////////////////////////////////////////////////////////////////////////
Subscriber::Subscriber(){

	// Read joy topic
	  sub_VelSub = nh_.subscribe<sensor_msgs::Joy>(SUB_TOPIC_NAME, SUB_TOPIC_BUFFER_SIZE, &Subscriber::joyCallback, this);
	// Publish Twist message with motorspeed
	  pub_VelPub = nh_.advertise<geometry_msgs::Twist>(PUB_TOPIC_NAME, PUB_TOPIC_BUFFER_SIZE);

	  //define values for safety
	  dScaleAxisX = 0;
	  dScaleAxisY = 0;
	  dScaleAxisZ = 0;

	//Check params x-axis, y-axis, z-axis
		if (nh_.hasParam("iAxisX"),nh_.hasParam("iAxisY"), nh_.hasParam("iAxisZ"))
		{
			nh_.getParam("iAxisX",iAxisX);
			nh_.getParam("iAxisY",iAxisY);
			nh_.getParam("iAxisZ",iAxisZ);
			ROS_INFO("The motorparameters are initialized with values: Motor1: %i, Motor2: %i, Motor3: %i.", iAxisX, iAxisY, iAxisZ);
		}
		else
		{
			ROS_ERROR_ONCE("The motorparameter(s) cannot be initialized");
			iAxisX=0; iAxisY=0, iAxisZ=0;
		}

	//Check params scaler x, y, z
		if (nh_.hasParam("dScaleAxisX"),nh_.hasParam("dScaleAxisY"),nh_.hasParam("dScaleAxisZ"))
		{
			nh_.getParam("dScaleAxisX",dScaleAxisX);
			nh_.getParam("dScaleAxisY",dScaleAxisY);
			nh_.getParam("dScaleAxisZ",dScaleAxisZ);
			ROS_INFO("The scalerparameters are initialized with values: Scaler1: %f, Scaler2: %f, Scaler3: %f.", dScaleAxisX, dScaleAxisY, dScaleAxisZ);
		}
		else
		{
			ROS_ERROR_ONCE("The scalerparameter(s) cannot be initialized");
			dScaleAxisX=0; dScaleAxisY=0, dScaleAxisZ=0;
		}
}

///////////////////////////////////////////////////////////////////////////////////////////
//Function: read joystick message.
//pre: 	
//post: 
/////////////////////////////////////////////////////////////////////////////////////////// 
void Subscriber::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	// read and calculate speedvalues
	dVelAxisX = dScaleAxisX*joy->axes[iAxisX];
	dVelAxisY = -dScaleAxisY*joy->axes[iAxisY]; //inverted for the right direction
	dVelAxisZ = -dScaleAxisZ*joy->axes[iAxisZ]; //inverted for the right direction
}

///////////////////////////////////////////////////////////////////////////////////////////
//Function: send twist value on ROS.
//pre: 	
//post: 
/////////////////////////////////////////////////////////////////////////////////////////// 
void Subscriber::sendValue(){
	//put speedvalues into vector

	msg.linear.x = dVelAxisX;
	msg.linear.y = dVelAxisY;
	msg.angular.z = dVelAxisZ;

	pub_VelPub.publish(msg);

	ROS_DEBUG("Actual motor set state: Motor1: %f, Motor2: %f, Motor3: %f.", dVelAxisX, dVelAxisY, dVelAxisZ);
	ROS_DEBUG("Actual scaler values: Scaler1: %f, Scaler2: %f, Scaler3: %f.", dScaleAxisX, dScaleAxisY, dScaleAxisZ);
}

