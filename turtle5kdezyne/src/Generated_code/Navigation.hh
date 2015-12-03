#ifndef NAVIGATION_HH
#define NAVIGATION_HH

#include "iNavigation.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct Navigation
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
  iNavigation My_Navigation;

  Navigation(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_Navigation_Navigate();
  returnResult::type My_Navigation_getCurrentLocation();
};

#endif // NAVIGATION_HH
