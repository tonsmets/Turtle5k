#include "Tactics.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


Tactics::Tactics(const dezyne::locator& dezyne_locator)
: dzn_meta("","Tactics",0)
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
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iControl::check_bindings,&My_Control)));
  dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&::iCommands::check_bindings,&My_Commands)));

  My_Control.meta.provides.port = "My_Control";
  My_Control.meta.provides.address = this;
  My_Control.meta.provides.meta = &this->dzn_meta;


  My_Commands.meta.requires.port = "My_Commands";
  My_Commands.meta.requires.address = this;
  My_Commands.meta.requires.meta = &this->dzn_meta;


  dzn_rt.performs_flush(this) = true;
  My_Control.in.tac_getTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Tactics,::iControl>,this,boost::function< returnResult::type()>(boost::bind(&Tactics::My_Control_tac_getTheBall,this)),boost::make_tuple(&My_Control, "tac_getTheBall", "return"));
  My_Control.in.tac_shootBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Tactics,::iControl>,this,boost::function< returnResult::type()>(boost::bind(&Tactics::My_Control_tac_shootBall,this)),boost::make_tuple(&My_Control, "tac_shootBall", "return"));
  My_Control.in.tac_attack = boost::bind(&dezyne::rcall_in< ::returnResult::type, Tactics,::iControl>,this,boost::function< returnResult::type()>(boost::bind(&Tactics::My_Control_tac_attack,this)),boost::make_tuple(&My_Control, "tac_attack", "return"));
  My_Control.in.tac_driveToTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Tactics,::iControl>,this,boost::function< returnResult::type()>(boost::bind(&Tactics::My_Control_tac_driveToTheBall,this)),boost::make_tuple(&My_Control, "tac_driveToTheBall", "return"));

}

returnResult::type Tactics::My_Control_tac_getTheBall()
{
  if (this->busy)
  {
    this->reply__returnResult = returnResult::busy;
  }
  else if (!(this->busy))
  {
    {
      this->returnCheck1 = this->My_Commands.in.findTheBall ();
      this->returnCheck2 = this->My_Commands.in.getToTheBall ();
      if ((this->returnCheck1 == returnResult::success && this->returnCheck2 == returnResult::success))
      this->reply__returnResult = returnResult::success;
      else
      this->reply__returnResult = returnResult::fail;
    }
  }
  return reply__returnResult;
}

returnResult::type Tactics::My_Control_tac_shootBall()
{
  if (this->busy)
  {
    this->reply__returnResult = returnResult::busy;
  }
  else if (!(this->busy))
  {
    {
      this->returnCheck1 = this->My_Commands.in.shootTheBall ();
      if (this->returnCheck1 == returnResult::success)
      this->reply__returnResult = returnResult::success;
      else
      this->reply__returnResult = returnResult::fail;
    }
  }
  return reply__returnResult;
}

returnResult::type Tactics::My_Control_tac_attack()
{
  if (this->busy)
  {
    this->reply__returnResult = returnResult::busy;
  }
  else if (!(this->busy))
  {
    {
      this->returnCheck1 = this->My_Commands.in.findTheBall ();
      this->returnCheck2 = this->My_Commands.in.getToTheBall ();
      this->returnCheck3 = this->My_Commands.in.shootTheBall ();
      if (((this->returnCheck1 == returnResult::success && this->returnCheck2 == returnResult::success) && this->returnCheck3 == returnResult::success))
      this->reply__returnResult = returnResult::success;
      else
      this->reply__returnResult = returnResult::fail;
    }
  }
  return reply__returnResult;
}

returnResult::type Tactics::My_Control_tac_driveToTheBall()
{
  if (!(this->busy))
  {
    {
      returnResult::type retCheck = returnResult::fail;
      retCheck = this->My_Commands.in.driveToTheBall ();
      if (((retCheck == returnResult::fail || retCheck == returnResult::busy) || retCheck == returnResult::success))
      this->reply__returnResult = retCheck;
    }
  }
  return reply__returnResult;
}


void Tactics::check_bindings() const
{
  dezyne::check_bindings(&dzn_meta);
}
void Tactics::dump_tree() const
{
  dezyne::dump_tree(&dzn_meta);
}

