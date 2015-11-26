#include "WorldModel_impl.hh"

#include <iostream>

returnResult::type WorldModel_impl::findTheBall()
{
	std::cout << "Enter returnresult for WorldModel_impl: 0 for y and 1 for no" << std::endl;
	int input=-1;
	scanf("%d", &input);
	if(input == 0)
	{
		return returnResult::type::success;
	}
	else
	{
		return returnResult::type::fail;
	}
}
