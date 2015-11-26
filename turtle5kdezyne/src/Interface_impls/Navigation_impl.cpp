#include "Navigation_impl.hh"

#include <iostream>

returnResult::type Navigation_impl::Navigate(float& x, float& y, float& z)
{
	std::cout << "Received x: " << x << " | y: " << y << " | z: " << z;
	return returnResult::type::success;
}
