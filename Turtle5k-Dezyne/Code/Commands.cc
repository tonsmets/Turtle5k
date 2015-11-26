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
#include "Commands.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


Commands::Commands(const dezyne::locator& dezyne_locator)
: dzn_meta{"","Commands",reinterpret_cast<const dezyne::component*>(this),0,{},{[this]{My_Commands.check_bindings();},[this]{My_BallControl.check_bindings();},[this]{My_WheelControl.check_bindings();},[this]{My_PositioningControl.check_bindings();}}}
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_Commands{{{"My_Commands",this},{"",0}}}
, My_BallControl{{{"",0},{"My_BallControl",this}}}
, My_WheelControl{{{"",0},{"My_WheelControl",this}}}
, My_PositioningControl{{{"",0},{"My_PositioningControl",this}}}
{
  dzn_rt.performs_flush(this) = true;
  My_Commands.in.findTheBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return My_Commands_findTheBall();}), std::make_tuple(&My_Commands, "findTheBall", "return"));
  };
  My_Commands.in.getToTheBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return My_Commands_getToTheBall();}), std::make_tuple(&My_Commands, "getToTheBall", "return"));
  };
  My_Commands.in.shootTheBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return My_Commands_shootTheBall();}), std::make_tuple(&My_Commands, "shootTheBall", "return"));
  };

}

returnResult::type Commands::My_Commands_findTheBall()
{
  {
    returnResult::type x = returnResult::fail;
    x = this->My_PositioningControl.in.findTheBall ();
    if (x == returnResult::succes)
    reply__returnResult = returnResult::succes;
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
    if (result == returnResult::succes)
    reply__returnResult = returnResult::succes;
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
    if (result == returnResult::succes)
    reply__returnResult = returnResult::succes;
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

