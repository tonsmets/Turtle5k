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
#include "BallControl.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


BallControl::BallControl(const dezyne::locator& dezyne_locator)
: dzn_meta{"","BallControl",reinterpret_cast<const dezyne::component*>(this),0,{},{[this]{BallControl.check_bindings();},[this]{BallHandling.check_bindings();},[this]{Shooting.check_bindings();}}}
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, BallControl{{{"BallControl",this},{"",0}}}
, BallHandling{{{"",0},{"BallHandling",this}}}
, Shooting{{{"",0},{"Shooting",this}}}
{
  dzn_rt.performs_flush(this) = true;
  BallControl.in.shootTheBall = [&] () {
    return dezyne::call_in(this, std::function<returnResult::type()>([&] {return BallControl_shootTheBall();}), std::make_tuple(&BallControl, "shootTheBall", "return"));
  };

}

returnResult::type BallControl::BallControl_shootTheBall()
{
  {
    returnResult::type result = returnResult::fail;
    result = this->Shooting.in.shootTheBall ();
    reply__returnResult = result;
  }
  return reply__returnResult;
}


void BallControl::check_bindings() const
{
  dezyne::check_bindings(reinterpret_cast<const dezyne::component*>(this));
}
void BallControl::dump_tree() const
{
  dezyne::dump_tree(reinterpret_cast<const dezyne::component*>(this));
}

