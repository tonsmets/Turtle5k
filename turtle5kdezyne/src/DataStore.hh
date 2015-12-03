#ifndef DATASTORE_HH
#define DATASTORE_HH

#include <geometry_msgs/Pose.h>

class DataStore
{
	public:
	DataStore();
	geometry_msgs::Pose ball_location;
	geometry_msgs::Pose robot_location;
};

#endif //DATASTORE_HH
