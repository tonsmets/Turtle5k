#include "WorldModel_impl.hh"

#include <iostream>

returnResult::type WorldModel_impl::findTheBall()
{
	std::cout << "Enter returnresult for WorldModel_impl: (y/n)" << std::endl;
	return returnResult::type::busy;
}
