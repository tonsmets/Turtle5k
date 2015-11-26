#include "locator.hh"
#include "runtime.hh"
#include "meta.hh"

#include "Robot.hh"

#include "WheelDriver_impl.hh"
#include "BallHandling_impl.hh"
#include "Navigation_impl.hh"
#include "Shooting_impl.hh"
#include "WorldModel_impl.hh"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <map>
#include <csignal>
#include <functional>
#include <memory>
#include <iostream>
#include <string>

#define PROGRAM_NAME "t5k_dezyne"
#define POSE_SUB_TOPIC "t5k/pose"

WorldModel_impl wm;
bool PoseLoaded = false;

void BindFunctions(Robot& robot)
{
	robot.My_WheelDriver.in.getToTheBall = WheelDriver_impl::getToTheBall;
	robot.My_BallHandling.in.handleTheBall = BallHandling_impl::handleTheBall;
	robot.My_Shooting.in.shootTheBall = Shooting_impl::shootTheBall;
	robot.My_WorldModel.in.findTheBall = WorldModel_impl::findTheBall;
	robot.My_Navigation.in.Navigate = Navigation_impl::Navigate;
}

void PoseCallback(const geometry_msgs::Pose::ConstPtr msg)
{
	std::cout << "PoseCallback called" << std::endl;
	WorldModel_impl::my_x = msg->position.x;
	WorldModel_impl::my_y = msg->position.y;
	WorldModel_impl::my_z = msg->position.z;
	PoseLoaded = true;
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, PROGRAM_NAME);
	ros::NodeHandle node;
	ros::Subscriber pose_sub = node.subscribe(POSE_SUB_TOPIC, 1000, PoseCallback);
	
	dezyne::locator locator;
	dezyne::runtime runtime;
	dezyne::port::meta meta;
	locator.set(runtime);
	locator.set(meta);
	
	Robot robot(locator);
	
	BindFunctions(robot);
	
	robot.check_bindings();

	while(ros::ok())
	{
		ros::spinOnce();
		if(PoseLoaded)
		{
			robot.My_Control.in.tac_getTheBall();
		}
	}

    return 0;
}
