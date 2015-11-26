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
#include "WheelControl.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


WheelControl::WheelControl(const dezyne::locator& dezyne_locator)
: dzn_meta{"","WheelControl",reinterpret_cast<const dezyne::component*>(this),0,{},{[this]{My_WheelControl.check_bindings();},[this]{My_WheelDriver.check_bindings();}}}
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_WheelControl{{{"My_WheelControl",this},{"",0}}}
, My_WheelDriver{{{"",0},{"My_WheelDriver",this}}}
{
  dzn_rt.performs_flush(this) = true;
  My_WheelControl.in.getToTheBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return My_WheelControl_getToTheBall();}), std::make_tuple(&My_WheelControl, "getToTheBall", "return"));
  };

}

returnResult::type WheelControl::My_WheelControl_getToTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->My_WheelDriver.in.getToTheBall ();
    reply__returnResult = result;
  }
  return reply__returnResult;
}


void WheelControl::check_bindings() const
{
  dezyne::check_bindings(reinterpret_cast<const dezyne::component*>(this));
}
void WheelControl::dump_tree() const
{
  dezyne::dump_tree(reinterpret_cast<const dezyne::component*>(this));
}

