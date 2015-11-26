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
: dzn_meta{"","PositioningControl",reinterpret_cast<const dezyne::component*>(this),0,{},{[this]{My_PositioningControl.check_bindings();},[this]{My_WorldModel.check_bindings();},[this]{My_Navigation.check_bindings();}}}
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_PositioningControl{{{"My_PositioningControl",this},{"",0}}}
, My_WorldModel{{{"",0},{"My_WorldModel",this}}}
, My_Navigation{{{"",0},{"My_Navigation",this}}}
{
  dzn_rt.performs_flush(this) = true;
  My_PositioningControl.in.findTheBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return My_PositioningControl_findTheBall();}), std::make_tuple(&My_PositioningControl, "findTheBall", "return"));
  };

}

returnResult::type PositioningControl::My_PositioningControl_findTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    float x;
    float y;
    float z;
    result = this->My_WorldModel.in.findTheBall (x, y, z);
    result = this->My_Navigation.in.Navigate (x, y, z);
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

