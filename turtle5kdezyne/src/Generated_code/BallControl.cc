#include "BallControl.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


BallControl::BallControl(const dezyne::locator& dezyne_locator)
: dzn_meta("","BallControl",0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_BallControl()
, My_BallHandling()
, My_Shooting()
{
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iBallControl::check_bindings,&My_BallControl)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iBallHandling::check_bindings,&My_BallHandling)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iShooting::check_bindings,&My_Shooting)));

  My_BallControl.meta.provides.port = "My_BallControl";
  My_BallControl.meta.provides.address = this;
  My_BallControl.meta.provides.meta = &this->dzn_meta;


  My_BallHandling.meta.requires.port = "My_BallHandling";
  My_BallHandling.meta.requires.address = this;
  My_BallHandling.meta.requires.meta = &this->dzn_meta;
  My_Shooting.meta.requires.port = "My_Shooting";
  My_Shooting.meta.requires.address = this;
  My_Shooting.meta.requires.meta = &this->dzn_meta;


  dzn_rt.performs_flush(this) = true;
  My_BallControl.in.shootTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, BallControl,::iBallControl>,this,boost::function< returnResult::type()>(boost::bind(&BallControl::My_BallControl_shootTheBall,this)),boost::make_tuple(&My_BallControl, "shootTheBall", "return"));

}

returnResult::type BallControl::My_BallControl_shootTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_Shooting.in.shootTheBall ();
    this->reply__returnResult = result;
  }
  return reply__returnResult;
}


void BallControl::check_bindings() const
{
  dezyne::check_bindings(&dzn_meta);
}
void BallControl::dump_tree() const
{
  dezyne::dump_tree(&dzn_meta);
}

