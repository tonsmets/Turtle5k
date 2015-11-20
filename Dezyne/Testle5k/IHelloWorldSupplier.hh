#ifndef IHELLOWORLDSUPPLIER_HH
#define IHELLOWORLDSUPPLIER_HH

#include "meta.hh"

#include <cassert>
#include <map>




struct IHelloWorldSupplier
{

  struct
  {
    std::function<void (std::string& value)> RequestString;
  } in;

  struct
  {
  } out;

  dezyne::port::meta meta;
  inline IHelloWorldSupplier(dezyne::port::meta m) : meta(m) {}

  void check_bindings() const
  {
    if (! in.RequestString) throw dezyne::binding_error_in(meta, "in.RequestString");


  }
};

inline void connect (IHelloWorldSupplier& provided, IHelloWorldSupplier& required)
{
  provided.out = required.out;
  required.in = provided.in;
  provided.meta.requires = required.meta.requires;
  required.meta.provides = provided.meta.provides;
}





#endif // IHELLOWORLDSUPPLIER_HH
