#include "Commands.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


Commands::Commands(const dezyne::locator& dezyne_locator)
: dzn_meta("","Commands",0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, posState(returnResult::fail)
, My_Commands()
, My_BallControl()
, My_DrivingControl()
, My_PositioningControl()
{
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iCommands::check_bindings,&My_Commands)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iBallControl::check_bindings,&My_BallControl)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iDrivingControl::check_bindings,&My_DrivingControl)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iPositioningControl::check_bindings,&My_PositioningControl)));

  My_Commands.meta.provides.port = "My_Commands";
  My_Commands.meta.provides.address = this;
  My_Commands.meta.provides.meta = &this->dzn_meta;


  My_BallControl.meta.requires.port = "My_BallControl";
  My_BallControl.meta.requires.address = this;
  My_BallControl.meta.requires.meta = &this->dzn_meta;
  My_DrivingControl.meta.requires.port = "My_DrivingControl";
  My_DrivingControl.meta.requires.address = this;
  My_DrivingControl.meta.requires.meta = &this->dzn_meta;
  My_PositioningControl.meta.requires.port = "My_PositioningControl";
  My_PositioningControl.meta.requires.address = this;
  My_PositioningControl.meta.requires.meta = &this->dzn_meta;


  dzn_rt.performs_flush(this) = true;
  My_Commands.in.findTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Commands,::iCommands>,this,boost::function< returnResult::type()>(boost::bind(&Commands::My_Commands_findTheBall,this)),boost::make_tuple(&My_Commands, "findTheBall", "return"));
  My_Commands.in.getToTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Commands,::iCommands>,this,boost::function< returnResult::type()>(boost::bind(&Commands::My_Commands_getToTheBall,this)),boost::make_tuple(&My_Commands, "getToTheBall", "return"));
  My_Commands.in.shootTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Commands,::iCommands>,this,boost::function< returnResult::type()>(boost::bind(&Commands::My_Commands_shootTheBall,this)),boost::make_tuple(&My_Commands, "shootTheBall", "return"));
  My_Commands.in.getCurrentLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, Commands,::iCommands>,this,boost::function< returnResult::type()>(boost::bind(&Commands::My_Commands_getCurrentLocation,this)),boost::make_tuple(&My_Commands, "getCurrentLocation", "return"));
  My_Commands.in.driveToLocation = boost::bind(&dezyne::rcall_in< ::returnResult::type, Commands,::iCommands>,this,boost::function< returnResult::type()>(boost::bind(&Commands::My_Commands_driveToLocation,this)),boost::make_tuple(&My_Commands, "driveToLocation", "return"));
  My_Commands.in.driveToTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Commands,::iCommands>,this,boost::function< returnResult::type()>(boost::bind(&Commands::My_Commands_driveToTheBall,this)),boost::make_tuple(&My_Commands, "driveToTheBall", "return"));

}

returnResult::type Commands::My_Commands_findTheBall()
{
  {
    returnResult::type x = returnResult::fail;
    x = this->My_PositioningControl.in.findTheBall ();
    if (x == returnResult::success)
    this->reply__returnResult = returnResult::success;
    else
    this->reply__returnResult = returnResult::fail;
  }
  return reply__returnResult;
}

returnResult::type Commands::My_Commands_getToTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_DrivingControl.in.getToTheBall ();
    if (result == returnResult::success)
    this->reply__returnResult = returnResult::success;
    else
    this->reply__returnResult = returnResult::fail;
  }
  return reply__returnResult;
}

returnResult::type Commands::My_Commands_shootTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_BallControl.in.shootTheBall ();
    if (result == returnResult::success)
    this->reply__returnResult = returnResult::success;
    else
    this->reply__returnResult = returnResult::fail;
  }
  return reply__returnResult;
}

returnResult::type Commands::My_Commands_getCurrentLocation()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_PositioningControl.in.getCurrentLocation ();
    this->reply__returnResult = result;
  }
  return reply__returnResult;
}

returnResult::type Commands::My_Commands_driveToLocation()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_DrivingControl.in.driveToLocation ();
    this->reply__returnResult = result;
  }
  return reply__returnResult;
}

returnResult::type Commands::My_Commands_driveToTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    if (this->posState == returnResult::busy)
    {
      result = this->My_PositioningControl.in.determinePath ();
      this->posState = result;
      this->reply__returnResult = result;
    }
    else
    if (this->posState == returnResult::fail)
    {
      result = this->My_PositioningControl.in.updatePositions ();
      if (result == returnResult::fail)
      this->reply__returnResult = result;
      else
      {
        result = this->My_PositioningControl.in.determinePath ();
        this->posState = result;
        this->reply__returnResult = result;
      }
    }
    else
    if (this->posState == returnResult::success)
    {
      result = this->My_DrivingControl.in.followPath ();
      if (result == returnResult::success)
      this->posState = returnResult::fail;
      this->reply__returnResult = result;
    }
    else
    {
      returnResult::type result = returnResult::fail;
      this->posState = result;
      this->reply__returnResult = result;
    }
  }
  return reply__returnResult;
}


void Commands::check_bindings() const
{
  dezyne::check_bindings(&dzn_meta);
}
void Commands::dump_tree() const
{
  dezyne::dump_tree(&dzn_meta);
}

