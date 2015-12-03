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
      busy, success, fail, yes, no, stub
    };
  };
#endif // ENUM__returnResult
  ::returnResult::type reply__returnResult;
  iCommands My_Commands;
  iBallControl My_BallControl;
  iWheelControl My_WheelControl;
  iPositioningControl My_PositioningControl;

  Commands(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_Commands_findTheBall();
  returnResult::type My_Commands_getToTheBall();
  returnResult::type My_Commands_shootTheBall();
};

#endif // COMMANDS_HH
