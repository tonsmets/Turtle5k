#include "WheelControl.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


WheelControl::WheelControl(const dezyne::locator& dezyne_locator)
: dzn_meta("","WheelControl",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_WheelControl()
, My_WheelDriver()
{
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWheelControl::check_bindings,&My_WheelControl)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWheelDriver::check_bindings,&My_WheelDriver)));

  My_WheelControl.meta.provides.port = "My_WheelControl";
  My_WheelControl.meta.provides.address = this;


  My_WheelDriver.meta.requires.port = "My_WheelDriver";
  My_WheelDriver.meta.requires.address = this;


  dzn_rt.performs_flush(this) = true;
  My_WheelControl.in.getToTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, WheelControl,iWheelControl>,this,boost::function< returnResult::type()>(boost::bind(&WheelControl::My_WheelControl_getToTheBall,this)),boost::make_tuple(&My_WheelControl, "getToTheBall", "return"));
  My_WheelControl.in.driveToLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, WheelControl,iWheelControl>,this,boost::function< returnResult::type()>(boost::bind(&WheelControl::My_WheelControl_driveToLocation,this)),boost::make_tuple(&My_WheelControl, "driveToLocation", "return"));

}

returnResult::type WheelControl::My_WheelControl_getToTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WheelDriver.in.getToTheBall ();
    reply__returnResult = result;
  }
  return reply__returnResult;
}

returnResult::type WheelControl::My_WheelControl_driveToLocation()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WheelDriver.in.driveToLocation ();
    reply__returnResult = result;
  }
  return reply__returnResult;
}


void WheelControl::check_bindings() const
{
  dezyne::check_bindings(reinterpret_cast<const dezyne::component*>(this));
}
void WheelControl::dump_tree() const
{
  dezyne::dump_tree(reinterpret_cast<const dezyne::component*>(this));
}

