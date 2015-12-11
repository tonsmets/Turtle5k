#include "WheelDriver.hh"
#include "DataStore.hh"
#include <angles/angles.h>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"
#include <cstdio>

#define DEFAULT_KA 1.6
#define DEFAULT_KB -0.5
#define DEFAULT_KP 0.6

ros::Publisher wd_pub;
DataStore * wd_ds;

WheelDriver::WheelDriver(const dezyne::locator& dezyne_locator)
: dzn_meta("","WheelDriver",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_WheelDriver()
{
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWheelDriver::check_bindings,&My_WheelDriver)));
	My_WheelDriver.meta.provides.port = "My_WheelDriver";
	My_WheelDriver.meta.provides.address = this;

	ros::NodeHandle& node = dzn_locator.get<ros::NodeHandle>();
	wd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	wd_ds = &(dzn_locator.get<DataStore>());
	
	dzn_rt.performs_flush(this) = true;
	
	My_WheelDriver.in.getToTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, WheelDriver, iWheelDriver>,this,boost::function< returnResult::type()>(boost::bind(&WheelDriver::My_WheelDriver_getToTheBall,this)),boost::make_tuple(&My_WheelDriver, "getToTheBall", "return"));
	My_WheelDriver.in.driveToLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, WheelDriver, iWheelDriver>,this,boost::function< returnResult::type()>(boost::bind(&WheelDriver::My_WheelDriver_driveToLocation,this)),boost::make_tuple(&My_WheelDriver, "driveToLocation", "return"));

}

returnResult::type WheelDriver::My_WheelDriver_getToTheBall()
{
	reply__returnResult = returnResult::fail;
	return reply__returnResult;
}

returnResult::type WheelDriver::My_WheelDriver_driveToLocation()
{
	geometry_msgs::Twist vel_msg;
	geometry_msgs::Pose goal_location = wd_ds->ball_location;
	nav_msgs::Odometry cur_location = wd_ds->robot_location;
	
	float x1 = goal_location.position.x - cur_location.pose.pose.position.x;
	float y1 = goal_location.position.y - cur_location.pose.pose.position.y;

	float alpha = angles::normalize_angle(atan2(y1, x1) - tf::getYaw(cur_location.pose.pose.orientation));
	float beta = angles::normalize_angle(tf::getYaw(goal_location.orientation) - atan2(y1, x1));

	vel_msg.angular.z = (1.6 * alpha) + (-0.5 * beta);
	vel_msg.linear.x = 0.6 * sqrt(pow(x1, 2) + pow(y1, 2));
	
	if(isnan(vel_msg.angular.z))
	{
		vel_msg.angular.z = 0;
	}

	wd_pub.publish(vel_msg);
	
	reply__returnResult = returnResult::busy;
	return reply__returnResult;
}
