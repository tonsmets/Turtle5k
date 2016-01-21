#include "WheelControl.hh"
#include "DataStore.hh"
#include <angles/angles.h>
#include <tf/tf.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Twist.h"
#include <cstdio>

#define DEFAULT_KA 1.6
#define DEFAULT_KB -0.5
#define DEFAULT_KP 0.6
#define LINEAR_ACCURACY_THRESHOLD 0.08
#define ANGULAR_ACCURACY_THRESHOLD 0.05

ros::Publisher wc_pub;
DataStore * wc_ds;

WheelControl::WheelControl(const dezyne::locator& dezyne_locator)
: dzn_meta("","WheelControl",0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_WheelControl()
{
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWheelControl::check_bindings,&My_WheelControl)));
	My_WheelControl.meta.provides.port = "My_WheelControl";
	My_WheelControl.meta.provides.address = this;

	ros::NodeHandle& node = dzn_locator.get<ros::NodeHandle>();
	wc_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	wc_ds = &(dzn_locator.get<DataStore>());
	
	dzn_rt.performs_flush(this) = true;
	
	My_WheelControl.in.getToTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, WheelControl, iWheelControl>,this,boost::function< returnResult::type()>(boost::bind(&WheelControl::My_WheelControl_getToTheBall,this)),boost::make_tuple(&My_WheelControl, "getToTheBall", "return"));
	My_WheelControl.in.driveToLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, WheelControl, iWheelControl>,this,boost::function< returnResult::type()>(boost::bind(&WheelControl::My_WheelControl_driveToLocation,this)),boost::make_tuple(&My_WheelControl, "driveToLocation", "return"));
	My_WheelControl.in.drivePathFromNavigation = boost::bind(&dezyne::rcall_in< ::returnResult::type, WheelControl, iWheelControl>,this,boost::function< returnResult::type()>(boost::bind(&WheelControl::My_WheelControl_drivePathFromNavigation, this)), boost::make_tuple(&My_WheelControl, "drivePathFromNavigation", "return"));
}

returnResult::type WheelControl::My_WheelControl_drivePathFromNavigation()
{
	geometry_msgs::Twist vel_msg;	
	nav_msgs::Odometry current_location = wc_ds->robot_location;
	std::vector<geometry_msgs::Pose> path(wc_ds->path);
	
	//Pathfollowing
	float x1 = path.at(1).position.x - current_location.pose.pose.position.x;
	float y1 = path.at(1).position.y - current_location.pose.pose.position.y;

	float alpha = angles::normalize_angle(atan2(y1, x1) - tf::getYaw(current_location.pose.pose.orientation));
	float beta = angles::normalize_angle(tf::getYaw(path.at(1).orientation) - atan2(y1, x1));

	vel_msg.angular.z = (1.6 * alpha) + (-0.5 * beta);
	vel_msg.linear.x = 0.6 * sqrt(pow(x1, 2) + pow(y1, 2));
	
	if(isnan(vel_msg.angular.z))
	{
		vel_msg.angular.z = 0;
	}

	if(vel_msg.linear.x < LINEAR_ACCURACY_THRESHOLD && fabs(vel_msg.angular.z) < ANGULAR_ACCURACY_THRESHOLD)
	{
		reply__returnResult = returnResult::success;
		return reply__returnResult;
	}
	else
	{
		wc_pub.publish(vel_msg);
		reply__returnResult = returnResult::busy;
		return reply__returnResult;
	}
}

returnResult::type WheelControl::My_WheelControl_getToTheBall()
{
	reply__returnResult = returnResult::fail;
	return reply__returnResult;
}

returnResult::type WheelControl::My_WheelControl_driveToLocation()
{
	reply__returnResult = returnResult::fail;
	return reply__returnResult;
}
