#include "DrivingControl.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


DrivingControl::DrivingControl(const dezyne::locator& dezyne_locator)
: dzn_meta("","DrivingControl",0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_DrivingControl()
, My_WheelControl()
{
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iDrivingControl::check_bindings,&My_DrivingControl)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iWheelControl::check_bindings,&My_WheelControl)));

  My_DrivingControl.meta.provides.port = "My_DrivingControl";
  My_DrivingControl.meta.provides.address = this;
  My_DrivingControl.meta.provides.meta = &this->dzn_meta;


  My_WheelControl.meta.requires.port = "My_WheelControl";
  My_WheelControl.meta.requires.address = this;
  My_WheelControl.meta.requires.meta = &this->dzn_meta;


  dzn_rt.performs_flush(this) = true;
  My_DrivingControl.in.getToTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, DrivingControl,::iDrivingControl>,this,boost::function< returnResult::type()>(boost::bind(&DrivingControl::My_DrivingControl_getToTheBall,this)),boost::make_tuple(&My_DrivingControl, "getToTheBall", "return"));
  My_DrivingControl.in.followPath = boost::bind(&dezyne::rcall_in< ::returnResult::type, DrivingControl,::iDrivingControl>,this,boost::function< returnResult::type()>(boost::bind(&DrivingControl::My_DrivingControl_followPath,this)),boost::make_tuple(&My_DrivingControl, "followPath", "return"));
  My_DrivingControl.in.driveToLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, DrivingControl,::iDrivingControl>,this,boost::function< returnResult::type()>(boost::bind(&DrivingControl::My_DrivingControl_driveToLocation,this)),boost::make_tuple(&My_DrivingControl, "driveToLocation", "return"));

}

returnResult::type DrivingControl::My_DrivingControl_getToTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WheelControl.in.getToTheBall ();
    this->reply__returnResult = result;
  }
  return reply__returnResult;
}

returnResult::type DrivingControl::My_DrivingControl_followPath()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WheelControl.in.drivePathFromNavigation ();
    this->reply__returnResult = result;
  }
  return reply__returnResult;
}

returnResult::type DrivingControl::My_DrivingControl_driveToLocation()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WheelControl.in.driveToLocation ();
    this->reply__returnResult = result;
  }
  return reply__returnResult;
}


void DrivingControl::check_bindings() const
{
  dezyne::check_bindings(&dzn_meta);
}
void DrivingControl::dump_tree() const
{
  dezyne::dump_tree(&dzn_meta);
}

