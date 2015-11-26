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
#ifndef WHEELCONTROL_HH
#define WHEELCONTROL_HH

#include "iWheelControl.hh"
#include "iWheelDriver.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct WheelControl
{
  dezyne::meta dzn_meta;
  dezyne::runtime& dzn_rt;
  dezyne::locator const& dzn_locator;
#ifndef ENUM__returnResult
#define ENUM__returnResult 1
  struct returnResult
  {
    enum type
    {
      busy, success, fail, yes, no, stub
    };
  };
#endif // ENUM__returnResult
  ::returnResult::type reply__returnResult;
  iWheelControl My_WheelControl;
  iWheelDriver My_WheelDriver;

  WheelControl(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_WheelControl_getToTheBall();
};

#endif // WHEELCONTROL_HH
