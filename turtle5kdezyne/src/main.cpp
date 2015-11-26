#include "locator.hh"
#include "runtime.hh"
#include "meta.hh"

#include "Robot.hh"

#include "WheelDriver_impl.hh"
#include "BallHandling_impl.hh"
#include "Navigation_impl.hh"
#include "Shooting_impl.hh"
#include "WorldModel_impl.hh"

#include <map>
#include <csignal>
#include <functional>
#include <memory>
#include <iostream>
#include <string>

void BindFunctions(Robot& robot)
{
	robot.My_WheelDriver.in.getToTheBall = WheelDriver_impl::getToTheBall;
	robot.My_BallHandling.in.handleTheBall = BallHandling_impl::handleTheBall;
	robot.My_Shooting.in.shootTheBall = Shooting_impl::shootTheBall;
	robot.My_WorldModel.in.findTheBall = WorldModel_impl::findTheBall;
	robot.My_Navigation.in.Navigate = Navigation_impl::Navigate;
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
