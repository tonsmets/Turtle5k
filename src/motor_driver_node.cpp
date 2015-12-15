/*****************************************************************************************************************************************
Function: 
This program responds on the receiving of a Float32MultiArray message on topic written in define variable "TOPIC_NAME.
The only data that this program will read is data[4], data[6] and data[7]. This is because serial port 5,7,8 will be written.
When the program can't open a serial port the program will be shut down. 
For debugging it is recommend to set the "log level" of ROS to -> DEBUG. normally it is setted to INFO.

Pre:
A "ros param" that descibes the factor between the incomming array (RPM) and the sending velocity to the motordrivers.
two "ros params" that describe the minimum and the maximum of the velocity that will be send to the motordrivers. NOTE: this is no RPM!!!

Post:
rs422 message on serial bus 5, 6 and 7 that gives the speed RPM. The exact message can be found in the outputbuffer. The encoder data will be
send on a Float32MultiArray. the data of port 5 is in array[4] etc.. The topic of this message = motorspeed_feedback.

Writer 		: Niek Francke
date 		: 11-12-2015
********************************************************************************************************************************************/

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include "ros/ros.h"
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

//-----settings
#define ROS_LOOP_RATE_HZ			1000
#define MOTORDRIVER_START_OF_FRAME 	0x5a
#define MOTORDRIVER_TYPE 			0xaa
#define MOTORDRIVER_TYPE_RECEIVE	0x55
#define MOTORDRIVER_CMD				0x03
#define MOTORDRIVER_CMD_RECEIVE		0x01
#define MOTORDRIVER_END_OF_FRAME	0x00
#define TOPIC_NAME 					"mcWheelVelocityMps"
#define TOPIC_BUFFER_SIZE			1
#define PUB_TOPIC_NAME				"motorspeed_feedback"
#define PUB_TOPIC_BUFFER_SIZE		1

#define OPEN_WITH_NONBLOCK 			1	
#define READ_RS422_ON				1	//Read port on/off
#define RS422_VMIN					0	//minimal number of characters for receiving data. only when OPEN_WITH_NONBLOCK = 1
#define RS422_VTIME					0	//time to wait on a signal. only when OPEN_WITH_NONBLOCK = 1
#define READ_RS422_INTERVAL_HZ		100	
#define DEBUG_SPEED					100 //delay for debug messages. it has no time defenition

#define MOTORDRIVER_KP_DEFAULT		100	//= kp * 100
#define MOTORDRIVER_KI_DEFAULT		0	//= ki * 100

#define SERIAL_PORT_5				4
#define SERIAL_PORT_7				6
#define SERIAL_PORT_8				7

using namespace std;

struct termios oldtio, newtio;       //place for old and new port settings for serial port
struct termios oldkey, newkey;       //place tor old and new port settings for keyboard teletype

/*****************************************************************************************************************************************
Start defining class Subscribe
********************************************************************************************************************************************/
class Subscribe
{	
public:
	struct Output{ char cOutBuf[8]; int iSpeed; char cInBuf[8];};	//data for serialports
	Output serialPorts[10];							
	int iConvertFactor;
	int iMaxPulseSpeed; 		//maximum RPM value
	int iMinPulseSpeed;			//Minimum RPM value
	int iSerialPortId[10];		//variable that catches the number of serial ports.	
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//Function: create class and read params
	//pre: 	-
	//post: iConvertFactor will be 0 if there is no param read.
	///////////////////////////////////////////////////////////////////////////////////////////
	Subscribe(ros::NodeHandle nh)
	{
		sub = nh.subscribe(TOPIC_NAME,TOPIC_BUFFER_SIZE, &Subscribe::commandRpmReceived, this);

		pub = nh.advertise<std_msgs::Float32MultiArray>(PUB_TOPIC_NAME, PUB_TOPIC_BUFFER_SIZE);

		iCommandReceivedCounter = 0;	//set counter to zero at the start of a project
		iEncoderDataReceiverCounter = 0;
		iSendError = 0;
		iFirstSendError = 0;
		iReceiveError = 0;
		iFirstReceiveError = 0;
		
		//Reading param. This param will be used to convert the data from RPM to pulses.
		std::string sParamName = "iConvertFactor";
		//check if param is defined.
		if (nh.hasParam(sParamName)){
			nh.getParam(sParamName, iConvertFactor);
			ROS_INFO("parameter iConvertFactor is read and has value :%i", iConvertFactor);
		}else{
			ROS_ERROR("Parameter iConvertFactor does not exist");
			iConvertFactor = 0;
		}

		//Reading param. This param will be used to convert the data from RPM to pulses.
		sParamName = "iMaxPulseSpeed";
		//check if param is defined.
		if (nh.hasParam(sParamName)){
			nh.getParam(sParamName, iMaxPulseSpeed);
			ROS_INFO("parameter iMaxPulseSpeed is read and has value :%i", iMaxPulseSpeed);
		}else{
			ROS_ERROR("Parameter iMaxPulseSpeed does not exist");
			iMaxPulseSpeed = 0;
		}

		//Reading param. This param will be used to convert the data from RPM to pulses.
		sParamName = "iMinPulseSpeed";
		//check if param is defined.
		if (nh.hasParam(sParamName)){
			nh.getParam(sParamName, iMinPulseSpeed);
			ROS_INFO("parameter iMinPulseSpeed is read and has value :%i", iMinPulseSpeed);
		}else{
			ROS_ERROR("Parameter iMinPulseSpeed does not exist");
			iMinPulseSpeed = 0;
		}

		//Reading param. This param will be used to convert the data from RPM to pulses.
		sParamName = "iOmniWheelMotorDriversPfactor";
		//check if param is defined.
		if (nh.hasParam(sParamName)){
			nh.getParam(sParamName, iKp);
			ROS_INFO("parameter iOmniWheelMotorDriversPfactor is read and has value :%i", iKp);
		}else{
			ROS_ERROR("Parameter iOmniWheelMotorDriversPfactor does not exist. Default value is taken");
			iKp = MOTORDRIVER_KP_DEFAULT;
		}

				//Reading param. This param will be used to convert the data from RPM to pulses.
		sParamName = "iOmniWheelMotorDriversIfactor";
		//check if param is defined.
		if (nh.hasParam(sParamName)){
			nh.getParam(sParamName, iKi);
			ROS_INFO("parameter iOmniWheelMotorDriversIfactor is read and has value :%i", iKi);
		}else{
			ROS_ERROR("Parameter iOmniWheelMotorDriversIfactor does not exist. Default value is taken");
			iKi = MOTORDRIVER_KI_DEFAULT;
		}
	}
	
	/////////////////////////////////////////////////////////////////////////////
	//function: this function responds on a float32MultiArray message.
	//pre: values for array places 4,6,7, because the serial ports are 5,7 and 8
	//post: -
	/////////////////////////////////////////////////////////////////////////////
	void commandRpmReceived(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		iCommandReceivedCounter++;

		ROS_INFO_ONCE("first time a FLoat32MultiArray is received");

		//print
		if(iCommandReceivedCounter % DEBUG_SPEED == 0){

			ROS_DEBUG("%i times Float32MultiArray received", iCommandReceivedCounter);

			//prints the data from FLoat32MultiArray to log
			for(int i = 0; i < 10 ; i++){
				ROS_DEBUG("data poort %i = %f" , i, msg->data[i]);
			}
		}

		//writes the data from Float32MultiArray to iSpeed for serialport.
		for(int i = 0 ; i < 10 ; i++){
			serialPorts[i].iSpeed = (int)msg->data[i];

			//overload protection
			if(serialPorts[i].iSpeed >= iMaxPulseSpeed){
				serialPorts[i].iSpeed = iMaxPulseSpeed;
				if(iCommandReceivedCounter % DEBUG_SPEED == 0) ROS_DEBUG("Speed is above maximum speed");
			}
			if(serialPorts[i].iSpeed <= iMinPulseSpeed){
				serialPorts[i].iSpeed = iMinPulseSpeed;
				if(iCommandReceivedCounter % DEBUG_SPEED == 0) ROS_DEBUG("Speed is below minimum speed");
			}
		}	
		
		//Make the messages for RS422 (motordrivers)	
		for(int i = 0 ; i < 10 ; i++){
			serialPorts[i].cOutBuf[0] = MOTORDRIVER_START_OF_FRAME;
			serialPorts[i].cOutBuf[1] = MOTORDRIVER_TYPE; 
			serialPorts[i].cOutBuf[2] = MOTORDRIVER_CMD; 
			serialPorts[i].cOutBuf[3] = (serialPorts[i].iSpeed & 0xff);
			serialPorts[i].cOutBuf[4] = (serialPorts[i].iSpeed >> 8) & 0xff;
			serialPorts[i].cOutBuf[5] = iKi;  //ki value motordrivers
			serialPorts[i].cOutBuf[6] = iKp;  //kp value motordrivers
			serialPorts[i].cOutBuf[7] = MOTORDRIVER_END_OF_FRAME;
		}

		//print speed data that will be send to motordrivers.
		if(iCommandReceivedCounter % DEBUG_SPEED == 0){
			for(int i = 0 ; i < 10 ; i++){
				//int iVelocity = (serialPorts[i].cOutBuf[4] << 8) + serialPorts[i].cOutBuf[3];
				ROS_DEBUG("data poort %i byte 3 = %i" , i,serialPorts[i].iSpeed);
				ROS_DEBUG("data poort %i byte 3 = %X" , i,serialPorts[i].cOutBuf[3]);
				ROS_DEBUG("data poort %i byte 4 = %X" , i,serialPorts[i].cOutBuf[4]);
			}	
		}

		int iNumberOfSendBytes, iWantedNumberOfSendBytes;
		//send data
		for(int i = 0; i < 3; i++){
			int iPort;

			//change values for serial ports
			switch(i){
				case 0: iPort = SERIAL_PORT_5; break;
				case 1: iPort = SERIAL_PORT_7; break;
				case 2: iPort = SERIAL_PORT_8; break;
			}

			iNumberOfSendBytes = write(iSerialPortId[iPort], serialPorts[iPort].cOutBuf,sizeof serialPorts[iPort].cOutBuf);
			iWantedNumberOfSendBytes = sizeof serialPorts[iPort].cOutBuf;

			//Write data.
			if(iWantedNumberOfSendBytes != iNumberOfSendBytes){
				ROS_ERROR("port %i gives a send error", (iPort+1));
				if(iSendError == 0){
					iFirstSendError = iCommandReceivedCounter;
				}
				iSendError++;
			}
		}

		if((iSendError != 0) && (iCommandReceivedCounter % DEBUG_SPEED == 0)){
				ROS_INFO("Total send errors = %i", iSendError);
				ROS_INFO("first pass with error send = %i", iFirstSendError);
				ROS_INFO("Number of wanted send bytes = %i", iWantedNumberOfSendBytes); 
				ROS_INFO("Number of send bytes = %i", iNumberOfSendBytes);
		}
	}

	/////////////////////////////////////////////////////////////////////////////
	//function: This function will read the RS422 ports and send the data to ROS
	//pre: 
	//post: values for array places 4,6,7, because the serial ports are 5,7 and 8
	/////////////////////////////////////////////////////////////////////////////
	bool readSerialPort(){

		iEncoderDataReceiverCounter++;

				//print
		if(iEncoderDataReceiverCounter % DEBUG_SPEED == 0) ROS_DEBUG("%i times encoder data received", iEncoderDataReceiverCounter);

		//create multiarray
		std_msgs::Float32MultiArray msg;
		

		int iNumberOfReceiveBytes, iWantedNumberOfReceiveBytes;
		//read data
		for(int x = 0; x < 3; x++){
			int iPort = 20; // value that has no port.
			
			//change values for serial ports
			switch(x){
				case 0: iPort = SERIAL_PORT_5; break;
				case 1: iPort = SERIAL_PORT_7; break;
				case 2: iPort = SERIAL_PORT_8; break;
			}

		iNumberOfReceiveBytes = read(iSerialPortId[iPort], serialPorts[iPort].cInBuf,sizeof serialPorts[iPort].cInBuf);
		iWantedNumberOfReceiveBytes = sizeof serialPorts[iPort].cInBuf;

		if ( iNumberOfReceiveBytes == iWantedNumberOfReceiveBytes){
				iSerialNewData[iPort] = true;	//is needed for sending data to ROS
			}else{
				ROS_ERROR("Encoder data read on port %i", (iPort + 1));
				if(iReceiveError == 0){
					iFirstReceiveError = iEncoderDataReceiverCounter;
				}
				iReceiveError++;
			}
		}


		if((iReceiveError != 0) && (iEncoderDataReceiverCounter % DEBUG_SPEED == 0)){
				ROS_INFO("Total receive errors = %i", iReceiveError);
				ROS_INFO("First pass with receive send = %i", iFirstReceiveError);
				ROS_INFO("Number of wanted receive bytes = %i", iWantedNumberOfReceiveBytes); 
				ROS_INFO("Number of received bytes = %i", iNumberOfReceiveBytes);
		}

		if(iSerialNewData[SERIAL_PORT_5] && iSerialNewData[SERIAL_PORT_7] && iSerialNewData[SERIAL_PORT_8]){	

			int16_t iEncoderData;

			if(iEncoderDataReceiverCounter % DEBUG_SPEED == 0){
				ROS_DEBUG("data encoder port 5:0x%x%x", serialPorts[SERIAL_PORT_5].cInBuf[4],serialPorts[SERIAL_PORT_5].cInBuf[3]);
				ROS_DEBUG("data encoder port 7:0x%x%x", serialPorts[SERIAL_PORT_7].cInBuf[4],serialPorts[SERIAL_PORT_7].cInBuf[3]);
				ROS_DEBUG("data encoder port 8:0x%x%x", serialPorts[SERIAL_PORT_8].cInBuf[4],serialPorts[SERIAL_PORT_8].cInBuf[3]);
			}



			bool bReadEnable[10];

			//check if received data is valid.
			for(int x = 0; x < 3; x++){
				int iPort = 20; // value that has no port.
				
				//change values for serial ports
				switch(x){
					case 0: iPort = SERIAL_PORT_5; break;
					case 1: iPort = SERIAL_PORT_7; break;
					case 2: iPort = SERIAL_PORT_8; break;
				}

				//check if a valid message is received.
				bReadEnable[iPort] = 	(serialPorts[iPort].cInBuf[0] == MOTORDRIVER_START_OF_FRAME) 
										&& (serialPorts[iPort].cInBuf[1] == MOTORDRIVER_TYPE_RECEIVE) 
										&& (serialPorts[iPort].cInBuf[2] == MOTORDRIVER_CMD_RECEIVE);
			}	

			//varify data and send it
			if( bReadEnable[SERIAL_PORT_5] && bReadEnable[SERIAL_PORT_7] && bReadEnable[SERIAL_PORT_8]){

				//put speedvalues into array
				msg.data.clear();
				msg.data.push_back(0);

				iEncoderData = ((unsigned char)serialPorts[SERIAL_PORT_5].cInBuf[6] << 8) | ((unsigned char)serialPorts[SERIAL_PORT_5].cInBuf[5]);
				ROS_INFO("encoder data2.5 = %i", iEncoderData);
				if(iEncoderDataReceiverCounter % DEBUG_SPEED == 0) ROS_INFO("data encoder wiel 5:%i", iEncoderData);
				msg.data.push_back(iEncoderData);

				iEncoderData = ((unsigned char)serialPorts[SERIAL_PORT_7].cInBuf[6] << 8) | ((unsigned char)serialPorts[SERIAL_PORT_7].cInBuf[5]);
				ROS_INFO("encoder data2.7 = %i", iEncoderData);
				if(iEncoderDataReceiverCounter % DEBUG_SPEED == 0) ROS_INFO("data encoder wiel 7:%i", iEncoderData);
				msg.data.push_back(iEncoderData);

				iEncoderData = ((unsigned char)serialPorts[SERIAL_PORT_8].cInBuf[6] << 8) | ((unsigned char)serialPorts[SERIAL_PORT_8].cInBuf[5]);
				ROS_INFO("encoder data2.8 = %i", iEncoderData);
				if(iEncoderDataReceiverCounter % DEBUG_SPEED == 0) ROS_INFO("data encoder wiel 5:%i", iEncoderData);
				msg.data.push_back(iEncoderData);

				iEncoderData = ((unsigned char)serialPorts[SERIAL_PORT_5].cInBuf[4] << 8) | ((unsigned char)serialPorts[SERIAL_PORT_5].cInBuf[3]);
				ROS_INFO("encoder data1.5 = %i", iEncoderData);
				if(iEncoderDataReceiverCounter % DEBUG_SPEED == 0) ROS_INFO("data encoder wiel 5:%i", iEncoderData);
				msg.data.push_back(iEncoderData);

				msg.data.push_back(0);

				iEncoderData = ((unsigned char)serialPorts[SERIAL_PORT_7].cInBuf[4] << 8) | ((unsigned char)serialPorts[SERIAL_PORT_7].cInBuf[3]);
				if(iEncoderDataReceiverCounter % DEBUG_SPEED == 0) ROS_INFO("data encoder wiel 7:%i", iEncoderData);
				msg.data.push_back(iEncoderData);

				ROS_INFO("encoder data1.7 = %i", iEncoderData);
				iEncoderData = ((unsigned char)serialPorts[SERIAL_PORT_8].cInBuf[4] << 8) | ((unsigned char)serialPorts[SERIAL_PORT_8].cInBuf[3]);		
				if(iEncoderDataReceiverCounter % DEBUG_SPEED == 0) ROS_INFO("data encoder wiel 8:%i", iEncoderData);
				msg.data.push_back(iEncoderData);

				ROS_INFO("encoder data1.8 = %i", iEncoderData);
				msg.data.push_back(0);

				//send message
				pub.publish(msg);
			}
			
			//clear markers
			for(int i = 0; i < 10 ; i++){
				iSerialNewData[i] = false;
			}
			
			if(iEncoderDataReceiverCounter % DEBUG_SPEED == 0) ROS_DEBUG("encoder data send");

			return 1;
		}else{
			return 0;
		}
	}

private:
	ros::Subscriber sub;	//define ros subscriber
	ros::Publisher	pub;	//define ros publisher
	bool iSerialNewData[10];

	//error detection
	int iCommandReceivedCounter; 		//counter for command received function
	int iEncoderDataReceiverCounter; 	//counter for command received function
	int iSendError;					 	
	int iFirstSendError;
	int iReceiveError;
	int iFirstReceiveError;

	int iKp;			//kp for motor drivers		
	int iKi;			//ki for motor drivers
};
/*****************************************************************************************************************************************
end of defining class Subscribe
********************************************************************************************************************************************/

/*****************************************************************************************************************************************
Start of main
********************************************************************************************************************************************/
int main(int argc, char **argv  )
{	
	//initialize ROS
	ros::init(argc, argv, "mcMotorDriver");
	
	//create nodehandle
	ros::NodeHandle nh;

	//create class
	Subscribe Sobject(nh);
	

	ROS_INFO("Serial Ports will be initialized");
	//ROS_DEBUG("O_NONBLOCK = %i", O_NONBLOCK);

	//open the serial ports and put the id number in a variable
	//O_RWDR = open for reading and writing
	//O_NOCTTY = the port never becomes the controlling terminal of the process
	//O_NDELAY = use non-blocking i/o. on some system this is also means the rs232 dcd signal line is ignored.	
	if(OPEN_WITH_NONBLOCK){
		Sobject.iSerialPortId[SERIAL_PORT_5] = open("/dev/ttyS5", O_RDWR | O_NONBLOCK);
		ROS_INFO("Serial port 5 are connected to hardware");
		Sobject.iSerialPortId[SERIAL_PORT_7] = open("/dev/ttyS7", O_RDWR | O_NONBLOCK);
		ROS_INFO("Serial port 7 are connected to hardware");
		Sobject.iSerialPortId[SERIAL_PORT_8] = open("/dev/ttyS8", O_RDWR | O_NONBLOCK);
		ROS_INFO("Serial port 8 are connected to hardware");
	}else{
		Sobject.iSerialPortId[SERIAL_PORT_5] = open("/dev/ttyS5", O_RDWR);
		ROS_INFO("Serial port 5 are connected to hardware");
		Sobject.iSerialPortId[SERIAL_PORT_7] = open("/dev/ttyS7", O_RDWR);
		ROS_INFO("Serial port 7 are connected to hardware");
		Sobject.iSerialPortId[SERIAL_PORT_8] = open("/dev/ttyS8", O_RDWR);
		ROS_INFO("Serial port 8 are connected to hardware");
	}

	for(int i = 0; i < 3; i++){
		int iPort;

		//change values for serial ports
		switch(i){
			case 0: iPort = SERIAL_PORT_5; break;
			case 1: iPort = SERIAL_PORT_7; break;
			case 2: iPort = SERIAL_PORT_8; break;
		}

		if(Sobject.iSerialPortId[iPort] <= 0){
			ROS_ERROR("Error opening port %i", iPort);
		}else{
				ROS_INFO("port %i is open", (iPort + 1));
			#if 1
				ROS_INFO("Setting options port %i", iPort);
				tcgetattr(Sobject.iSerialPortId[iPort],&oldkey); // save current port settings   //so commands are interpreted right for this program
				// set new port settings for non-canonical input processing  //must be NOCTTY
				newkey.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
				newkey.c_iflag = IGNPAR;
				newkey.c_oflag = 0;
				newkey.c_lflag = 0;       //ICANON;
				newkey.c_cc[VMIN]= RS422_VMIN;
				newkey.c_cc[VTIME]= RS422_VTIME;
				tcflush(Sobject.iSerialPortId[iPort], TCIFLUSH);
				tcsetattr(Sobject.iSerialPortId[iPort],TCSANOW,&newkey);
				ROS_INFO("Setting options completed for port %i", (iPort +1));
			#endif
		}
	}

	ROS_INFO("Serial ports are initialized");
	ROS_INFO("Starting Transmission");

	ros::Rate loop_rate(ROS_LOOP_RATE_HZ);
	int iWhileCounter = 0;

	while(ros::ok() )
	{	
		iWhileCounter++;

		//Choose with which rate the RS422 port will be read.
		//after 300 ms the ports start reading. otherwise there will be a lot of errors.
		if((iWhileCounter >  1000) && (iWhileCounter % (ROS_LOOP_RATE_HZ/READ_RS422_INTERVAL_HZ) == 0)){
			//check if read data is on
			if(READ_RS422_ON){
				Sobject.readSerialPort();
			}
		}

		//wait until a Float32MulitArray is received and run the callback function
		ros::spinOnce();
		loop_rate.sleep();
	}
		// here the port will be closed and the keyboard will be given back to the system.
		close(Sobject.iSerialPortId[SERIAL_PORT_5]);
		close(Sobject.iSerialPortId[SERIAL_PORT_7]);
		close(Sobject.iSerialPortId[SERIAL_PORT_8]);
		ROS_INFO("ports are closed\n");

	return 0;
}

/*****************************************************************************************************************************************
End of main
********************************************************************************************************************************************/