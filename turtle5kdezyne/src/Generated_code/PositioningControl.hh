#ifndef POSITIONINGCONTROL_HH
#define POSITIONINGCONTROL_HH

#include "iPositioningControl.hh"
#include "iWorldModel.hh"
#include "iNavigation.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct PositioningControl
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
  iPositioningControl My_PositioningControl;
  iWorldModel My_WorldModel;
  iNavigation My_Navigation;

  PositioningControl(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_PositioningControl_findTheBall();
  returnResult::type My_PositioningControl_getCurrentLocation();
  returnResult::type My_PositioningControl_updatePositions();
  returnResult::type My_PositioningControl_determinePath();
};

#endif // POSITIONINGCONTROL_HH
