#include "locator.hh"
#include "runtime.hh"
#include "meta.hh"

#include "Robot.hh"

#include "WheelDriver_impl.hh"

#include <map>
#include <csignal>
#include <functional>
#include <memory>
#include <iostream>
#include <string>

void BindFunctions(Robot& robot)
{
	robot.My_WheelDriver.in.getToTheBall = WheelDriver_impl::getToTheBall;
}

int main() {
	dezyne::locator locator;
	dezyne::runtime runtime;
	dezyne::port::meta meta;
	locator.set(runtime);
	locator.set(meta);
	
	Robot robot(locator);
	
	BindFunctions(robot);
	
	robot.check_bindings();
	robot.My_Control.in.tac_getTheBall();


	int dontclosewindow;
	std::cin >> dontclosewindow;

    return 0;
}
