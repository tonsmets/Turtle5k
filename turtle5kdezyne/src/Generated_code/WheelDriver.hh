#ifndef WHEELDRIVER_HH
#define WHEELDRIVER_HH

#include "iWheelDriver.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct WheelDriver
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
  iWheelDriver My_WheelDriver;

  WheelDriver(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_WheelDriver_getToTheBall();
  returnResult::type My_WheelDriver_driveToLocation();
};

#endif // WHEELDRIVER_HH
