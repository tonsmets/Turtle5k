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
: dzn_meta{"","WheelControl",reinterpret_cast<const dezyne::component*>(this),0,{},{[this]{WheelControl.check_bindings();},[this]{WheelDriver.check_bindings();}}}
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, WheelControl{{{"WheelControl",this},{"",0}}}
, WheelDriver{{{"",0},{"WheelDriver",this}}}
{
  dzn_rt.performs_flush(this) = true;
  WheelControl.in.getToTheBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return WheelControl_getToTheBall();}), std::make_tuple(&WheelControl, "getToTheBall", "return"));
  };

}

returnResult::type WheelControl::WheelControl_getToTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->WheelDriver.in.getToTheBall ();
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

