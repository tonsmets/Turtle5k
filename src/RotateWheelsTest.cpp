//	All the libs that we do and may need :
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <stdint.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stddef.h>
#include <sys/ioctl.h>

using namespace std;

#define MESSAGE_SIZE		8
#define PASS_COUNT			1000
#define SLEEP				1000
#define OPEN_WITH_NONBLOCK 	0
#define READ_ON				1

#define KP 					100
#define KI 					0

/* Call this just when main() does its initialization. */
/* Note: kbhit will call this if it hasn't been done yet. */
void kbinit();

int kbhit();

/* Call this just before main() quits, to restore TTY settings! */
void kbfini();

static int initialized = 0;
static struct termios original_tty;

/*******************************************************************************************************
********************************************************************************************************/
int main(int argc, char *argv[])
{
	//init variables
	char device_name[] = "/dev/ttyS8";
	int i = 0;
	int i2= 0;
	int send_error = 0;
	int receive_error = 0;
	int number_of_bytes_send, number_of_bytes_to_send;
	int number_of_bytes_receive, number_of_bytes_to_receive;
	int first_error_on_pass = 0;
	int first_error_onPass_receive = 0;
	int old_error_send = 0;
	int old_error_receive = 0;
	double rel_error;
	unsigned char received_bytes[8];
	int16_t wheelVel;

	struct termios oldtio, newtio;       //place for old and new port settings for serial port
   	struct termios oldkey, newkey;       //place tor old and new port settings for keyboard teletype
	
	printf("Enter wheel value: ");
	cin>>wheelVel;
	// -----------------------------------------------------------------------------------
	
	//	Check if the number is bigger or smaller than what we can actually send to the controler ( wheelVel >= 0x01 && wheelVel <= 0xFF ).
	if(wheelVel>1100) wheelVel=1100;
	if(wheelVel<1) wheelVel=1;
	// -----------------------------------------------------------------------------------
	
	//	These are the hex values that we send to the controller.
	//	The first 3 bytes ( 0x5a,0xaa,0x03 ) are used for checking purposes on the controller. If they are not right we can't procede further.
	// 	The 4th and the 5th bytes are the ones responsible for the speed and the direction of the rotation.
	static unsigned char send_bytes[MESSAGE_SIZE] = {0x5A, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
//	static unsigned char send_bytes[] = {0x5A, 0xAA, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	// -----------------------------------------------------------------------------------
	
	//	Here we open the devices connected to the two controllers.
	printf("Opening %s\n", device_name);

	int wheel1_open;

	if(OPEN_WITH_NONBLOCK){	
		wheel1_open = open(device_name, O_RDWR | O_NONBLOCK);
	}else{
		wheel1_open = open(device_name, O_RDWR); // changed by Gerard
	}

	if(wheel1_open <= 0){
		printf("Error opening %s\n", device_name);
	}
	else{
		printf("%s is open\n", device_name);
#if 1
		printf("Setting options\n");
		tcgetattr(wheel1_open,&oldkey); // save current port settings   //so commands are interpreted right for this program
		// set new port settings for non-canonical input processing  //must be NOCTTY
		newkey.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
		newkey.c_iflag = IGNPAR;
		newkey.c_oflag = 0;
		newkey.c_lflag = 0;       //ICANON;
		newkey.c_cc[VMIN]=8;
		newkey.c_cc[VTIME]=0;
		tcflush(wheel1_open, TCIFLUSH);
		tcsetattr(wheel1_open,TCSANOW,&newkey);
		printf("Setting options completed\n");
#endif
		printf("Starting Transmission\n");

		// -----------------------------------------------------------------------------------
	
		//	For now this loop is endless for testing purposes but the condition may be changed according to any requirements.


		while(!kbhit())	//while can be ended by pressing a button on the keyboard.
		{
			i++;

			//	Here we convert the wheelVel's value accordingly and then send it to both of the controllers.
			send_bytes[3]=(char)(wheelVel & 0xFF);
			send_bytes[4]=(char)((wheelVel >> 8) & 0xFF);
			//send_bytes[5]=(char)(KP & 0xFF);
			//send_bytes[6]=(char)(KI & 0xFF);
		
			//send
			number_of_bytes_to_send = sizeof(send_bytes);
			number_of_bytes_send = write(wheel1_open, send_bytes, number_of_bytes_to_send);				
		
			if(!(number_of_bytes_to_send == number_of_bytes_send)){
				if(send_error == 0){
					first_error_on_pass = i;
				}
				send_error++;
			}

			//the while loop needs to run a few time, because the first cycle there is no data
			if(READ_ON && (i > 5)){
				//receive
				number_of_bytes_to_receive = sizeof(received_bytes);

				number_of_bytes_receive = read(wheel1_open, received_bytes, sizeof received_bytes);

				if(!(number_of_bytes_to_receive == number_of_bytes_receive)){
					if(receive_error == 0){
						first_error_onPass_receive = i;
					}
					receive_error++;
					printf("number of bytes received in pass %i = %i", i ,number_of_bytes_receive);	
				}
			}
				
			//Print functions
			if((i % PASS_COUNT) == 0){
				printf("Pass = %i\n", i);
				printf("************SEND***********************\n");
				printf("Send error = %i\n", send_error);
				printf("First send error on pass = %i\n",  first_error_on_pass);
				rel_error = (((send_error - old_error_send) * 1.0)/(PASS_COUNT * 1.0)) * 100.0;
				printf("Number of send errors(relative) = %0.1f%%\n", rel_error);  
				old_error_send = send_error;

				if(READ_ON){
					printf("************RECEIVE***********************\n");
					printf("Receive error = %i\n", receive_error);
					printf("First receive error on pass = %i\n", first_error_onPass_receive);
					rel_error = (((receive_error - old_error_receive) * 1.0)/(PASS_COUNT * 1.0)) * 100.0;
					printf("Number of receive errors(relative) = %0.1f%%\n", rel_error);  
					old_error_receive = receive_error;

					printf("encoder data = 0x");
					for(int x = 0; x<8; x++){
						printf("%X", received_bytes[x]);
					}
					printf("\n");
				}
			}
			usleep(SLEEP);	//sleep function is defined
		}
	
		// here the port will be closed and the keyboard will be given back to the system.
		close(wheel1_open);
		printf("ports are closed\n");
		kbfini();
		printf("Keyboard settings are given back to system\n");
	}
	return 0;
}
/*****************************************************************************************************************
*******************************************************************************************************************/


/* Call this just when main() does its initialization. */
/* Note: kbhit will call this if it hasn't been done yet. */
void kbinit()
{
  struct termios tty;
  tcgetattr(fileno(stdin), &original_tty);
  tty = original_tty;

  /* Disable ICANON line buffering, and ECHO. */
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tcsetattr(fileno(stdin), TCSANOW, &tty);

  /* Decouple the FILE*'s internal buffer. */
  /* Rely on the OS buffer, probably 8192 bytes. */
  setbuf(stdin, NULL);
  initialized = 1;
}

int kbhit() 
{
  if(!initialized)
  {
    kbinit();
  }

  int bytesWaiting;
  ioctl(fileno(stdin), FIONREAD, &bytesWaiting);
  return bytesWaiting;
}

/* Call this just before main() quits, to restore TTY settings! */
void kbfini()
{
  if(initialized)
  {
    tcsetattr(fileno(stdin), TCSANOW, &original_tty);
    initialized = 0;
  }
}
