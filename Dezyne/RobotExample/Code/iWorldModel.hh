#ifndef IWORLDMODEL_HH
#define IWORLDMODEL_HH

#include "meta.hh"

#include <cassert>
#include <map>




struct iWorldModel
{

  struct
  {
    std::function<void ()> stub;
  } in;

  struct
  {
  } out;

  dezyne::port::meta meta;
  inline iWorldModel(dezyne::port::meta m) : meta(m) {}

  void check_bindings() const
  {
    if (! in.stub) throw dezyne::binding_error_in(meta, "in.stub");


  }
};

inline void connect (iWorldModel& provided, iWorldModel& required)
{
  provided.out = required.out;
  required.in = provided.in;
  provided.meta.requires = required.meta.requires;
  required.meta.provides = provided.meta.provides;
}





#endif // IWORLDMODEL_HH
