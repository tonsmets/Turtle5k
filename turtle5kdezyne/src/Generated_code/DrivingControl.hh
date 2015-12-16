#ifndef DRIVINGCONTROL_HH
#define DRIVINGCONTROL_HH

#include "iDrivingControl.hh"
#include "iWheelControl.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct DrivingControl
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
  returnResult::type reply__returnResult;
  iDrivingControl My_DrivingControl;
  iWheelControl My_WheelControl;

  DrivingControl(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_DrivingControl_getToTheBall();
  returnResult::type My_DrivingControl_followPath();
  returnResult::type My_DrivingControl_driveToLocation();
};

#endif // DRIVINGCONTROL_HH
