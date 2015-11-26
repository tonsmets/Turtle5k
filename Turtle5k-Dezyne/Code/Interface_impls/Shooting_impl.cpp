#include "Shooting_impl.hh"

#include <iostream>

returnResult::type Shooting_impl::shootTheBall()
{
	std::cout << "Enter returnresult for Shooting_impl: (y/n)" << std::endl;
	return returnResult::type::busy;
}
