#include 		<stdio.h>
#include 		<string.h>
#include 		<unistd.h>
#include 		<fcntl.h>
#include 		<errno.h>
#include 		<termios.h>
#include 		<time.h>
#include 		<stdint.h>
#include 		<iostream>
#include 		<cmath>
#include 		<stdlib.h>
#include		<ctime>
#include 		<time.h>
#include 		"/home/robocup/TurtleTest/Current Code/keydown/keydown.h"

//------------------------------------------------------------------------------------------

#define 		sensor_left 0x04
#define 		sensor_right 0x03
#define 		scale_left 8
#define 		scale_right 9
#define 		clockScale 5000
#define			norm_min_speed 30
#define			norm_max_speed 60
#define			hall_difference 0.7

//------------------------------------------------------------------------------------------

using namespace std;

//------------------------------------------------------------------------------------------

string 			devices[]={"/dev/ttyS4","/dev/ttyS6","/dev/ttyS9"};

static int 		ports_open[sizeof devices],
				level_left=0,
				level_right=0,
				degree_left=0,
				degree_right=0,
				//tacho_left=0,
				//tacho_right=0,
				clockCompare=0,
				//count=0,
				//go_angle=0;

static clock_t 	clockNow,
				clockPart;
				
static int16_t 	wheelVel_left,
				wheelVel_right;	
bool			have_ball=false;	

				
//------------------------------------------------------------------------------------------

void 			OpenPorts(string _devices[],int _ports_open[])
{
				for(int i=0;i<sizeof _devices;i++)
				_ports_open[i]  = open(_devices[i].c_str(), O_RDWR | O_NONBLOCK);
}

//------------------------------------------------------------------------------------------

void 			ClosePorts(int _ports_open[])
{
				for(int i=0;i<sizeof _ports_open;i++)
				close(_ports_open[i]);
}

//------------------------------------------------------------------------------------------

void 			ReadSensor	(int &level,int &degree,int scale,int port_open,unsigned char arm)
{
				unsigned char	send_bytes[] = 	{0x5A, 0x00, 0x00, 0x00, 0x00, 0x0d},
								data_buf[6];
				send_bytes[1]=arm;
	
				write(port_open, send_bytes, sizeof send_bytes);
				read(port_open, &data_buf, sizeof data_buf );
				
				level=(int)data_buf[3]-scale;
				degree=(int)data_buf[4]-scale;
}

//------------------------------------------------------------------------------------------

int16_t			CalculateSpeed(int level,int degree,int sign)
{
				
				int16_t speed=0,
						min_speed=norm_min_speed,
						max_speed=norm_max_speed;
				double 	scale;
				if(sign>0)
				scale=(degree/255)+level;
				else
				scale=(degree/255)+level+hall_difference;
					
				if(scale<4)
				{
					have_ball=true;
				}
				else
				{
					have_ball=false;
				}
					
				//if(scale<2)
				//{
					//min_speed=norm_min_speed;
					//max_speed=norm_max_speed;
				//}
				//else
				//{
					//if(scale>=2 && scale <=4)
					//{
						//min_speed=norm_min_speed*1.25;
						//max_speed=norm_max_speed*1.25;
					//}
					//else
					//{
						//min_speed=norm_min_speed*1.5;
						//max_speed=norm_max_speed*1.5;
					//}
				//}
				
				//if(go_angle==0)
				//{
					//speed=min_speed/2;
				//}
				//else
				//if(go_angle==90)
				//{
					//if(sign>0)
					//speed=max_speed;
					//else
					//speed=min_speed/2;
				//}
				//else
				//if(go_angle==180)
				//{
					//speed=max_speed;
				//}
				//else
				//if(go_angle==270)
				//{
					//if(sign<0)
					//speed=max_speed;
					//else
					//speed=min_speed/2;
				//}
				//cout<<have_ball<<endl;
				return sign*110;
}

//------------------------------------------------------------------------------------------

void 			SetSpeed	(int level,int degree,int port_open,int sign)
{				
				unsigned char	send_bytes[] = 	{0x5A, 0xAA, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00},
								data_buf[8];
				int16_t			wheelVel=0;
	
				wheelVel=CalculateSpeed(level,degree,sign);
		
				send_bytes[3]=(char)(wheelVel & 0xFF);
				send_bytes[4]=(char)((wheelVel >> 8) & 0xFF);
				
				write(port_open, send_bytes, sizeof send_bytes);
				//if(count==1000000)
				//{
				//read(port_open, &data_buf, sizeof data_buf);
				//count=0;
				//}
				//else count++;
}

//------------------------------------------------------------------------------------------

int 			main(int argc, char *argv[])
{
				OpenPorts(devices,ports_open);
				
				while(1)
				{
					clockNow=clock();
					
					if(clockPart==clockCompare)
					{
						clockCompare+=1;
						
						ReadSensor	(level_right,degree_right,scale_right,ports_open[0],sensor_right);
									
						ReadSensor	(level_left,degree_left,scale_left,ports_open[0],sensor_left);
					}
					else clockPart=clockNow/clockScale;	
					
					SetSpeed	(level_left,degree_left,ports_open[2],-1);
					
					SetSpeed	(level_right,degree_right,ports_open[1],1);
					
					if(kbhit()) break;
				}
				
				ClosePorts(ports_open);
	
				return 0;
}
