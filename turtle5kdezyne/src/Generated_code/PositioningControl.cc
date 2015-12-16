#include "PositioningControl.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


PositioningControl::PositioningControl(const dezyne::locator& dezyne_locator)
: dzn_meta("","PositioningControl",0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_PositioningControl()
, My_WorldModel()
, My_Navigation()
{
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iPositioningControl::check_bindings,&My_PositioningControl)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iWorldModel::check_bindings,&My_WorldModel)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iNavigation::check_bindings,&My_Navigation)));

  My_PositioningControl.meta.provides.port = "My_PositioningControl";
  My_PositioningControl.meta.provides.address = this;
  My_PositioningControl.meta.provides.meta = &this->dzn_meta;


  My_WorldModel.meta.requires.port = "My_WorldModel";
  My_WorldModel.meta.requires.address = this;
  My_WorldModel.meta.requires.meta = &this->dzn_meta;
  My_Navigation.meta.requires.port = "My_Navigation";
  My_Navigation.meta.requires.address = this;
  My_Navigation.meta.requires.meta = &this->dzn_meta;


  dzn_rt.performs_flush(this) = true;
  My_PositioningControl.in.findTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, PositioningControl,::iPositioningControl>,this,boost::function< returnResult::type()>(boost::bind(&PositioningControl::My_PositioningControl_findTheBall,this)),boost::make_tuple(&My_PositioningControl, "findTheBall", "return"));
  My_PositioningControl.in.getCurrentLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, PositioningControl,::iPositioningControl>,this,boost::function< returnResult::type()>(boost::bind(&PositioningControl::My_PositioningControl_getCurrentLocation,this)),boost::make_tuple(&My_PositioningControl, "getCurrentLocation", "return"));
  My_PositioningControl.in.updatePositions = boost::bind(&dezyne::rcall_in< ::returnResult::type, PositioningControl,::iPositioningControl>,this,boost::function< returnResult::type()>(boost::bind(&PositioningControl::My_PositioningControl_updatePositions,this)),boost::make_tuple(&My_PositioningControl, "updatePositions", "return"));
  My_PositioningControl.in.determinePath = boost::bind(&dezyne::rcall_in< ::returnResult::type, PositioningControl,::iPositioningControl>,this,boost::function< returnResult::type()>(boost::bind(&PositioningControl::My_PositioningControl_determinePath,this)),boost::make_tuple(&My_PositioningControl, "determinePath", "return"));

}

returnResult::type PositioningControl::My_PositioningControl_findTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WorldModel.in.getCurrentBallLocation ();
    this->reply__returnResult = result;
  }
  return reply__returnResult;
}

returnResult::type PositioningControl::My_PositioningControl_getCurrentLocation()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WorldModel.in.getCurrentRobotLocation ();
    this->reply__returnResult = result;
  }
  return reply__returnResult;
}

returnResult::type PositioningControl::My_PositioningControl_updatePositions()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WorldModel.in.isThereABall ();
    if (result == returnResult::success)
    {
      result = this->My_WorldModel.in.getCurrentBallLocation ();
      result = this->My_WorldModel.in.getCurrentRobotLocation ();
      this->reply__returnResult = result;
    }
    else
    this->reply__returnResult = returnResult::fail;
  }
  return reply__returnResult;
}

returnResult::type PositioningControl::My_PositioningControl_determinePath()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_Navigation.in.determinePath ();
    this->reply__returnResult = result;
  }
  return reply__returnResult;
}


void PositioningControl::check_bindings() const
{
  dezyne::check_bindings(&dzn_meta);
}
void PositioningControl::dump_tree() const
{
  dezyne::dump_tree(&dzn_meta);
}

