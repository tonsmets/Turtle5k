#include "BallHandling_impl.hh"

#include <iostream>

returnResult::type BallHandling_impl::handleTheBall()
{
	std::cout << "Enter returnresult for BallHandling_impl: (y/n)" << std::endl;
	return returnResult::type::busy;
}
