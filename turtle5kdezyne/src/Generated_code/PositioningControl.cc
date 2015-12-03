#include "PositioningControl.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


PositioningControl::PositioningControl(const dezyne::locator& dezyne_locator)
: dzn_meta("","PositioningControl",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_PositioningControl()
, My_WorldModel()
, My_Navigation()
{
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iPositioningControl::check_bindings,&My_PositioningControl)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWorldModel::check_bindings,&My_WorldModel)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iNavigation::check_bindings,&My_Navigation)));

  My_PositioningControl.meta.provides.port = "My_PositioningControl";
  My_PositioningControl.meta.provides.address = this;


  My_WorldModel.meta.requires.port = "My_WorldModel";
  My_WorldModel.meta.requires.address = this;
  My_Navigation.meta.requires.port = "My_Navigation";
  My_Navigation.meta.requires.address = this;


  dzn_rt.performs_flush(this) = true;
  My_PositioningControl.in.findTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, PositioningControl,iPositioningControl>,this,boost::function< returnResult::type()>(boost::bind(&PositioningControl::My_PositioningControl_findTheBall,this)),boost::make_tuple(&My_PositioningControl, "findTheBall", "return"));

}

returnResult::type PositioningControl::My_PositioningControl_findTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WorldModel.in.findTheBall ();
    result = this->My_Navigation.in.Navigate ();
    reply__returnResult = result;
  }
  return reply__returnResult;
}


void PositioningControl::check_bindings() const
{
  dezyne::check_bindings(reinterpret_cast<const dezyne::component*>(this));
}
void PositioningControl::dump_tree() const
{
  dezyne::dump_tree(reinterpret_cast<const dezyne::component*>(this));
}

