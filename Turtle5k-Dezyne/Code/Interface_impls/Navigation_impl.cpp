#include "Navigation_impl.hh"

#include <iostream>

returnResult::type Navigation_impl::Navigate()
{
	std::cout << "Enter returnresult for Navigation_impl: (y/n)" << std::endl;
	return returnResult::type::busy;
}
