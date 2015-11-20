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
#include "HelloWorldApplication.hh"

#include "locator.hh"
#include "runtime.hh"

#include <iostream>


HelloWorldApplication::HelloWorldApplication(const dezyne::locator& dezyne_locator)
: dzn_meta{"","HelloWorldApplication",reinterpret_cast<const dezyne::component*>(this),0,{},{[this]{app.check_bindings();},[this]{supplier.check_bindings();},[this]{printer.check_bindings();}}}
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, app{{{"app",this},{"",0}}}
, supplier{{{"",0},{"supplier",this}}}
, printer{{{"",0},{"printer",this}}}
{
  dzn_rt.performs_flush(this) = true;
  app.in.start = [&] () {
    dezyne::call_in(this, [this] {app_start();}, std::make_tuple(&app, "start", "return"));
  };

}

void HelloWorldApplication::app_start()
{
  {
    std::string value;
    this->supplier.in.RequestString(value);
    this->printer.in.PrintString(value);
  }
}


void HelloWorldApplication::check_bindings() const
{
  dezyne::check_bindings(reinterpret_cast<const dezyne::component*>(this));
}
void HelloWorldApplication::dump_tree() const
{
  dezyne::dump_tree(reinterpret_cast<const dezyne::component*>(this));
}

