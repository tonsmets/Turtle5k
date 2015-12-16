#include "WorldModel.hh"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <turtle5kdezyne/BallLocation.h> 
#include <tf/tf.h>
#include "DataStore.hh"


ros::Subscriber wm_ball_sub;
ros::Subscriber wm_robot_sub;
DataStore * wm_ds;

bool m_isThereABall = false;

void ballLocationCallback(const turtle5kdezyne::BallLocation::ConstPtr& msg);
void robotLocationCallback(const nav_msgs::Odometry msg);

WorldModel::WorldModel(const dezyne::locator& dezyne_locator)
: dzn_meta("","WorldModel",0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_WorldModel()
{
	ros::NodeHandle& node = dzn_locator.get<ros::NodeHandle>();
	wm_ball_sub = node.subscribe("/t5k/ball_location", 1000, ballLocationCallback);
	wm_robot_sub = node.subscribe("/odom", 1000, robotLocationCallback);
	wm_ds = &(dzn_locator.get<DataStore>());
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWorldModel::check_bindings,&My_WorldModel)));
	My_WorldModel.meta.provides.port = "WorldModel";
	My_WorldModel.meta.provides.address = this;
	
	dzn_rt.performs_flush(this) = true;
	
	My_WorldModel.in.isThereABall = boost::bind(&dezyne::rcall_in< ::returnResult::type, WorldModel, iWorldModel>,this,boost::function< returnResult::type()>(boost::bind(&WorldModel::My_WorldModel_isThereABall,this)),boost::make_tuple(&My_WorldModel, "isThereABall", "return"));
	My_WorldModel.in.getCurrentBallLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, WorldModel, iWorldModel>,this,boost::function< returnResult::type()>(boost::bind(&WorldModel::My_WorldModel_getCurrentBallLocation,this)),boost::make_tuple(&My_WorldModel, "getCurrentBallLocation", "return"));
	My_WorldModel.in.getCurrentRobotLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, WorldModel, iWorldModel>,this,boost::function< returnResult::type()>(boost::bind(&WorldModel::My_WorldModel_getCurrentRobotLocation,this)),boost::make_tuple(&My_WorldModel, "getCurrentRobotLocation", "return"));
}

void robotLocationCallback(const nav_msgs::Odometry msg)
{
	nav_msgs::Odometry tmp = msg;
	try
	{
		tf::assertQuaternionValid(tmp.pose.pose.orientation);
	}
	catch(tf::InvalidArgument & iaex)
	{
		//Implement a way to make sure the quaternion is valid or the driving part doesnt do anything with it
		tmp.pose.pose.orientation.x = 0;
		tmp.pose.pose.orientation.y = 0;
		tmp.pose.pose.orientation.z = 0;		
		tmp.pose.pose.orientation.w = 1;
	}
	wm_ds->robot_location = tmp;
}

void ballLocationCallback(const turtle5kdezyne::BallLocation::ConstPtr& msg)
{
	std::cout << "BallLocationCallback called" << std::endl;
	wm_ds->ball_location = msg->pose.pose;
	m_isThereABall = msg->found;
}

returnResult::type WorldModel::My_WorldModel_getCurrentRobotLocation()
{
	reply__returnResult == returnResult::success;
	return reply__returnResult;
}

returnResult::type WorldModel::My_WorldModel_getCurrentBallLocation()
{
	reply__returnResult = returnResult::success;
	return reply__returnResult;
}


returnResult::type WorldModel::My_WorldModel_isThereABall()
{
	//Change implementation to allow for a check that the data is loaded in the DS
	if(m_isThereABall == true)
	{	
		reply__returnResult = returnResult::success;
	}
	else
	{
		reply__returnResult = returnResult::fail;
	}
	return reply__returnResult;
}
