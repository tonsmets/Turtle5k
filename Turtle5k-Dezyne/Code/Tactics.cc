// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "Tactics.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


Tactics::Tactics(const dezyne::locator& dezyne_locator)
: dzn_meta{"","Tactics",reinterpret_cast<const dezyne::component*>(this),0,{},{[this]{Control.check_bindings();},[this]{Commands.check_bindings();}}}
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, busy(false)
, failure(false)
, returnCheck1(returnResult::succes)
, returnCheck2(returnResult::succes)
, returnCheck3(returnResult::succes)
, Control{{{"Control",this},{"",0}}}
, Commands{{{"",0},{"Commands",this}}}
{
  dzn_rt.performs_flush(this) = true;
  Control.in.tac_getTheBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return Control_tac_getTheBall();}), std::make_tuple(&Control, "tac_getTheBall", "return"));
  };
  Control.in.tac_shootBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return Control_tac_shootBall();}), std::make_tuple(&Control, "tac_shootBall", "return"));
  };
  Control.in.tac_attack = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return Control_tac_attack();}), std::make_tuple(&Control, "tac_attack", "return"));
  };

}

returnResult::type Tactics::Control_tac_getTheBall()
{
  if (busy)
  {
    reply__returnResult = returnResult::busy;
  }
  else if (! (busy)
  )
  {
    {
      returnCheck1 = this->Commands.in.findTheBall ();
      returnCheck2 = this->Commands.in.getToTheBall ();
      if (returnCheck1 == returnResult::succes && returnCheck2 == returnResult::succes
      && returnCheck3 == returnResult::succes
      )
      reply__returnResult = returnResult::succes;
      else
      reply__returnResult = returnResult::fail;
    }
  }
  return reply__returnResult;
}

returnResult::type Tactics::Control_tac_shootBall()
{
  if (busy)
  {
    reply__returnResult = returnResult::busy;
  }
  else if (! (busy)
  )
  {
    {
      returnCheck1 = this->Commands.in.shootTheBall ();
      if (returnCheck1 == returnResult::succes && returnCheck2 == returnResult::succes
      && returnCheck3 == returnResult::succes
      )
      reply__returnResult = returnResult::succes;
      else
      reply__returnResult = returnResult::fail;
    }
  }
  return reply__returnResult;
}

returnResult::type Tactics::Control_tac_attack()
{
  if (busy)
  {
    reply__returnResult = returnResult::busy;
  }
  else if (! (busy)
  )
  {
    {
      returnCheck1 = this->Commands.in.findTheBall ();
      returnCheck2 = this->Commands.in.getToTheBall ();
      returnCheck3 = this->Commands.in.shootTheBall ();
      if (returnCheck1 == returnResult::succes && returnCheck2 == returnResult::succes
      && returnCheck3 == returnResult::succes
      )
      reply__returnResult = returnResult::succes;
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
