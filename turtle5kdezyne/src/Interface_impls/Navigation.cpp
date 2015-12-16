#include "Navigation.hh"
#include "DataStore.hh"
#include <ros/ros.h>


ros::Subscriber nav_sub;
DataStore * nav_ds;

void robotLocationCallback(const nav_msgs::Odometry msg);

Navigation::Navigation(const dezyne::locator& dezyne_locator)
: dzn_meta("","Navigation",0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_Navigation()
{
	ros::NodeHandle& node = dzn_locator.get<ros::NodeHandle>();
	nav_ds = &(dzn_locator.get<DataStore>());
	
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iNavigation::check_bindings,&My_Navigation)));
	My_Navigation.meta.provides.port = "My_Navigation";
	My_Navigation.meta.provides.address = this;
	
	dzn_rt.performs_flush(this) = true;
	
	My_Navigation.in.determinePath = boost::bind(&dezyne::rcall_in< ::returnResult::type, Navigation, iNavigation>, this, boost::function< returnResult::type()>(boost::bind(&Navigation::My_Navigation_determinePath, this)), boost::make_tuple(&My_Navigation, "determinePath", "return"));	
}

returnResult::type Navigation::My_Navigation_determinePath()
{
	//Here you would insert either your own path planning or a call to a stack that does it for you, thats why the busy is implemented
	nav_ds->path.clear();
	nav_ds->path.push_back(nav_ds->robot_location.pose.pose);
	nav_ds->path.push_back(nav_ds->ball_location);
	
	reply__returnResult = returnResult::success;
	return reply__returnResult;
}

