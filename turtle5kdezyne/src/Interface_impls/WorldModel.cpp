#include "WorldModel.hh"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "DataStore.hh"


ros::Subscriber wm_sub;
DataStore * wm_ds;

void ballLocationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);


WorldModel::WorldModel(const dezyne::locator& dezyne_locator)
: dzn_meta("","WorldModel",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_WorldModel()
{
	ros::NodeHandle& node = dzn_locator.get<ros::NodeHandle>();
	wm_sub = node.subscribe("t5k/ball_location", 1000, ballLocationCallback);
	wm_ds = &(dzn_locator.get<DataStore>());
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWorldModel::check_bindings,&My_WorldModel)));
	My_WorldModel.meta.provides.port = "WorldModel";
	My_WorldModel.meta.provides.address = this;
	
	dzn_rt.performs_flush(this) = true;
	
	My_WorldModel.in.findTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, WorldModel, iWorldModel>,this,boost::function< returnResult::type()>(boost::bind(&WorldModel::My_WorldModel_findTheBall,this)),boost::make_tuple(&My_WorldModel, "findTheBall", "return"));
}

void ballLocationCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	std::cout << "BallLocationCallback called" << std::endl;
	wm_ds->ball_location = msg->pose;
}

returnResult::type WorldModel::My_WorldModel_findTheBall()
{
	//Chance implementation to allow for a check that the data is loaded in the DS
	reply__returnResult = returnResult::success;
	return reply__returnResult;
}
