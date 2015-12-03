#ifndef BALLHANDLING_HH
#define BALLHANDLING_HH

#include "iBallHandling.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct BallHandling
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
  iBallHandling My_BallHandling;

  BallHandling(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_BallHandling_handleTheBall();
};

#endif // BALLHANDLING_HH
