#ifndef BALLCONTROL_HH
#define BALLCONTROL_HH

#include "iBallControl.hh"
#include "iBallHandling.hh"
#include "iShooting.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct BallControl
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
  iBallControl My_BallControl;
  iBallHandling My_BallHandling;
  iShooting My_Shooting;

  BallControl(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_BallControl_shootTheBall();
};

#endif // BALLCONTROL_HH
