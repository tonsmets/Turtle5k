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
#ifndef COMMANDS_HH
#define COMMANDS_HH

#include "iCommands.hh"
#include "iBallControl.hh"
#include "iWheelControl.hh"
#include "iPositioningControl.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct Commands
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
      busy, succes, fail, yes, no, stub
    };
  };
#endif // ENUM__returnResult
  ::returnResult::type reply__returnResult;
  iCommands Commands;
  iBallControl BallControl;
  iWheelControl WheelControl;
  iPositioningControl PositioningControl;

  Commands(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type Commands_findTheBall();
  returnResult::type Commands_getToTheBall();
  returnResult::type Commands_shootTheBall();
};

#endif // COMMANDS_HH
