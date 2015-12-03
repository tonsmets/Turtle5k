#ifndef TACTICS_HH
#define TACTICS_HH

#include "iControl.hh"
#include "iCommands.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct Tactics
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
  bool busy;
  bool failure;
  returnResult::type returnCheck1;
  returnResult::type returnCheck2;
  returnResult::type returnCheck3;
  ::returnResult::type reply__returnResult;
  iControl My_Control;
  iCommands My_Commands;

  Tactics(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_Control_tac_getTheBall();
  returnResult::type My_Control_tac_shootBall();
  returnResult::type My_Control_tac_attack();
};

#endif // TACTICS_HH
