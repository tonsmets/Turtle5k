#include "WheelDriver_impl.hh"

#include <iostream>

returnResult::type WheelDriver_impl::getToTheBall()
{
	std::cout << "Enter returnresult for WheelDriver_impl: (y/n)" << std::endl;
	return returnResult::type::busy;
}
