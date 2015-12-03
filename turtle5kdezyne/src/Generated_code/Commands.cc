#include "Commands.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


Commands::Commands(const dezyne::locator& dezyne_locator)
: dzn_meta("","Commands",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_Commands()
, My_BallControl()
, My_WheelControl()
, My_PositioningControl()
{
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iCommands::check_bindings,&My_Commands)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iBallControl::check_bindings,&My_BallControl)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWheelControl::check_bindings,&My_WheelControl)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iPositioningControl::check_bindings,&My_PositioningControl)));

  My_Commands.meta.provides.port = "My_Commands";
  My_Commands.meta.provides.address = this;


  My_BallControl.meta.requires.port = "My_BallControl";
  My_BallControl.meta.requires.address = this;
  My_WheelControl.meta.requires.port = "My_WheelControl";
  My_WheelControl.meta.requires.address = this;
  My_PositioningControl.meta.requires.port = "My_PositioningControl";
  My_PositioningControl.meta.requires.address = this;


  dzn_rt.performs_flush(this) = true;
  My_Commands.in.findTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Commands,iCommands>,this,boost::function< returnResult::type()>(boost::bind(&Commands::My_Commands_findTheBall,this)),boost::make_tuple(&My_Commands, "findTheBall", "return"));
  My_Commands.in.getToTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Commands,iCommands>,this,boost::function< returnResult::type()>(boost::bind(&Commands::My_Commands_getToTheBall,this)),boost::make_tuple(&My_Commands, "getToTheBall", "return"));
  My_Commands.in.shootTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Commands,iCommands>,this,boost::function< returnResult::type()>(boost::bind(&Commands::My_Commands_shootTheBall,this)),boost::make_tuple(&My_Commands, "shootTheBall", "return"));

}

returnResult::type Commands::My_Commands_findTheBall()
{
  {
    returnResult::type x = returnResult::fail;
    x = this->My_PositioningControl.in.findTheBall ();
    if (x == returnResult::success)
    reply__returnResult = returnResult::success;
    else
    reply__returnResult = returnResult::fail;
  }
  return reply__returnResult;
}

returnResult::type Commands::My_Commands_getToTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WheelControl.in.getToTheBall ();
    if (result == returnResult::success)
    reply__returnResult = returnResult::success;
    else
    reply__returnResult = returnResult::fail;
  }
  return reply__returnResult;
}

returnResult::type Commands::My_Commands_shootTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_BallControl.in.shootTheBall ();
    if (result == returnResult::success)
    reply__returnResult = returnResult::success;
    else
    reply__returnResult = returnResult::fail;
  }
  return reply__returnResult;
}


void Commands::check_bindings() const
{
  dezyne::check_bindings(reinterpret_cast<const dezyne::component*>(this));
}
void Commands::dump_tree() const
{
  dezyne::dump_tree(reinterpret_cast<const dezyne::component*>(this));
}

