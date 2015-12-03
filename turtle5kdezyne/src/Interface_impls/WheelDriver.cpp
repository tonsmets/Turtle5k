#include "WheelDriver.hh"

#include <cstdio>

WheelDriver::WheelDriver(const dezyne::locator& dezyne_locator)
: dzn_meta("","WheelDriver",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_WheelDriver()
{
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWheelDriver::check_bindings,&My_WheelDriver)));
	My_WheelDriver.meta.provides.port = "My_WheelDriver";
	My_WheelDriver.meta.provides.address = this;
	
	dzn_rt.performs_flush(this) = true;
	
	My_WheelDriver.in.getToTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, WheelDriver, iWheelDriver>,this,boost::function< returnResult::type()>(boost::bind(&WheelDriver::My_WheelDriver_getToTheBall,this)),boost::make_tuple(&My_WheelDriver, "getToTheBall", "return"));
}

returnResult::type WheelDriver::My_WheelDriver_getToTheBall()
{
	std::cout << "Enter returnresult for WheelDriver_impl: 0 for y and 1 for no" << std::endl;
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
