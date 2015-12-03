#include "Tactics.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


Tactics::Tactics(const dezyne::locator& dezyne_locator)
: dzn_meta("","Tactics",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, busy(false)
, failure(false)
, returnCheck1(returnResult::success)
, returnCheck2(returnResult::success)
, returnCheck3(returnResult::success)
, My_Control()
, My_Commands()
{
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iControl::check_bindings,&My_Control)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iCommands::check_bindings,&My_Commands)));

  My_Control.meta.provides.port = "My_Control";
  My_Control.meta.provides.address = this;


  My_Commands.meta.requires.port = "My_Commands";
  My_Commands.meta.requires.address = this;


  dzn_rt.performs_flush(this) = true;
  My_Control.in.tac_getTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Tactics,iControl>,this,boost::function< returnResult::type()>(boost::bind(&Tactics::My_Control_tac_getTheBall,this)),boost::make_tuple(&My_Control, "tac_getTheBall", "return"));
  My_Control.in.tac_shootBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Tactics,iControl>,this,boost::function< returnResult::type()>(boost::bind(&Tactics::My_Control_tac_shootBall,this)),boost::make_tuple(&My_Control, "tac_shootBall", "return"));
  My_Control.in.tac_attack = boost::bind(&dezyne::rcall_in< ::returnResult::type, Tactics,iControl>,this,boost::function< returnResult::type()>(boost::bind(&Tactics::My_Control_tac_attack,this)),boost::make_tuple(&My_Control, "tac_attack", "return"));

}

returnResult::type Tactics::My_Control_tac_getTheBall()
{
  if (busy)
  {
    reply__returnResult = returnResult::busy;
  }
  else if (not (busy))
  {
    {
      returnCheck1 = this->My_Commands.in.findTheBall ();
      returnCheck2 = this->My_Commands.in.getToTheBall ();
      if (returnCheck1 == returnResult::success and returnCheck2 == returnResult::success)
      reply__returnResult = returnResult::success;
      else
      reply__returnResult = returnResult::fail;
    }
  }
  return reply__returnResult;
}

returnResult::type Tactics::My_Control_tac_shootBall()
{
  if (busy)
  {
    reply__returnResult = returnResult::busy;
  }
  else if (not (busy))
  {
    {
      returnCheck1 = this->My_Commands.in.shootTheBall ();
      if (returnCheck1 == returnResult::success)
      reply__returnResult = returnResult::success;
      else
      reply__returnResult = returnResult::fail;
    }
  }
  return reply__returnResult;
}

returnResult::type Tactics::My_Control_tac_attack()
{
  if (busy)
  {
    reply__returnResult = returnResult::busy;
  }
  else if (not (busy))
  {
    {
      returnCheck1 = this->My_Commands.in.findTheBall ();
      returnCheck2 = this->My_Commands.in.getToTheBall ();
      returnCheck3 = this->My_Commands.in.shootTheBall ();
      if (returnCheck1 == returnResult::success and returnCheck2 == returnResult::success and returnCheck3 == returnResult::success)
      reply__returnResult = returnResult::success;
      else
      reply__returnResult = returnResult::fail;
    }
  }
  return reply__returnResult;
}


void Tactics::check_bindings() const
{
  dezyne::check_bindings(reinterpret_cast<const dezyne::component*>(this));
}
void Tactics::dump_tree() const
{
  dezyne::dump_tree(reinterpret_cast<const dezyne::component*>(this));
}

