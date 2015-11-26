#include "WorldModel_impl.hh"

#include <iostream>

float WorldModel_impl::my_x;
float WorldModel_impl::my_y;
float WorldModel_impl::my_z;

WorldModel_impl::WorldModel_impl()
{
	my_x = 0.0;
	my_y = 0.0;
	my_z = 0.0;
}


returnResult::type WorldModel_impl::findTheBall(float& x, float& y, float& z)
{
	std::cout << "WorldModel_impl:: Put the last received values into float references" << std::endl;
	x = my_x;
	y = my_y;
	z = my_z;
	return returnResult::type::success;
}
