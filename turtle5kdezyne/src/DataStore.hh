#ifndef DATASTORE_HH
#define DATASTORE_HH

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

class DataStore
{
	public:
	DataStore();
	geometry_msgs::Pose ball_location;
	nav_msgs::Odometry robot_location;
};

#endif //DATASTORE_HH
