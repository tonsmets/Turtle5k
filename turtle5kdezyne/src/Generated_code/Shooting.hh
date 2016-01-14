#ifndef SHOOTING_HH
#define SHOOTING_HH

#include "iShooting.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct Shooting
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
  iShooting My_Shooting;

  Shooting(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_Shooting_shootTheBall();
};

#endif // SHOOTING_HH
