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
#include "PositioningControl.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


PositioningControl::PositioningControl(const dezyne::locator& dezyne_locator)
: dzn_meta{"","PositioningControl",reinterpret_cast<const dezyne::component*>(this),0,{},{[this]{PositioningControl.check_bindings();},[this]{WorldModel.check_bindings();},[this]{Navigation.check_bindings();}}}
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, PositioningControl{{{"PositioningControl",this},{"",0}}}
, WorldModel{{{"",0},{"WorldModel",this}}}
, Navigation{{{"",0},{"Navigation",this}}}
{
  dzn_rt.performs_flush(this) = true;
  PositioningControl.in.findTheBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return PositioningControl_findTheBall();}), std::make_tuple(&PositioningControl, "findTheBall", "return"));
  };

}

returnResult::type PositioningControl::PositioningControl_findTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->WorldModel.in.findTheBall ();
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

