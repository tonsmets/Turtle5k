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
