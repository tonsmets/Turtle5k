#include "BallHandling.hh"

#include <cstdio>

BallHandling::BallHandling(const dezyne::locator& dezyne_locator)
: dzn_meta("","BallHandling",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_BallHandling()
{
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iBallHandling::check_bindings,&My_BallHandling)));
	My_BallHandling.meta.provides.port = "My_BallHandling";
	My_BallHandling.meta.provides.address = this;
	
	dzn_rt.performs_flush(this) = true;
	
	My_BallHandling.in.handleTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, BallHandling, iBallHandling>,this,boost::function< returnResult::type()>(boost::bind(&BallHandling::My_BallHandling_handleTheBall,this)),boost::make_tuple(&My_BallHandling, "handleTheBall", "return"));
}

returnResult::type BallHandling::My_BallHandling_handleTheBall()
{
	std::cout << "Enter returnresult for BallHandling: 0 for y and 1 for no" << std::endl;
	int input=-1;
	std::scanf("%d", &input);
	if(input == 0)
	{
		reply__returnResult = returnResult::success;
	}
	else
	{
		reply__returnResult = returnResult::fail;
	}
	return reply__returnResult;
}
