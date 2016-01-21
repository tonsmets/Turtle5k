#ifndef WORLDMODEL_HH
#define WORLDMODEL_HH

#include "iWorldModel.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct WorldModel
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
  iWorldModel My_WorldModel;

  WorldModel(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  returnResult::type My_WorldModel_isThereABall();
  returnResult::type My_WorldModel_getCurrentBallLocation();
  returnResult::type My_WorldModel_getCurrentRobotLocation();
};

#endif // WORLDMODEL_HH
