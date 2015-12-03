#include "Navigation.hh"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "DataStore.hh"

ros::Subscriber nav_sub;
DataStore * nav_ds;

void robotLocationCallback(const geometry_msgs::PoseStamped::ConstPtr msg);

Navigation::Navigation(const dezyne::locator& dezyne_locator)
: dzn_meta("","Navigation",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_Navigation()
{
	ros::NodeHandle& node = dzn_locator.get<ros::NodeHandle>();
	nav_sub = node.subscribe("t5k/robot_location", 1000, robotLocationCallback);
	nav_ds = &(dzn_locator.get<DataStore>());
	
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iNavigation::check_bindings,&My_Navigation)));
	My_Navigation.meta.provides.port = "My_Navigation";
	My_Navigation.meta.provides.address = this;
	
	dzn_rt.performs_flush(this) = true;
	
	My_Navigation.in.Navigate = boost::bind(&dezyne::rcall_in< ::returnResult::type, Navigation, iNavigation>,this,boost::function< returnResult::type()>(boost::bind(&Navigation::My_Navigation_Navigate,this)),boost::make_tuple(&My_Navigation, "Navigate", "return"));
	My_Navigation.in.getCurrentLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, Navigation, iNavigation>,this,boost::function< returnResult::type()>(boost::bind(&Navigation::My_Navigation_getCurrentLocation,this)),boost::make_tuple(&My_Navigation, "getCurrentLocation", "return"));
}

void robotLocationCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	nav_ds->robot_location = msg->pose;
}

returnResult::type Navigation::My_Navigation_Navigate()
{
	reply__returnResult = returnResult::fail;
	return reply__returnResult;
}

returnResult::type Navigation::My_Navigation_getCurrentLocation()
{
	reply__returnResult = returnResult::fail;
	return reply__returnResult;
}
