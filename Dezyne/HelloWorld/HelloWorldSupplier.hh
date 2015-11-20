#ifndef HELLOWORLDSUPPLIER_HH
#define HELLOWORLDSUPPLIER_HH

#include "meta.hh"

#include <cassert>
#include <map>




struct HelloWorldSupplier
{

  struct
  {
    std::function<void (std::string& value)> RequestString;
  } in;

  struct
  {
  } out;

  dezyne::port::meta meta;
  inline HelloWorldSupplier(dezyne::port::meta m) : meta(m) {}

  void check_bindings() const
  {
    if (! in.RequestString) throw dezyne::binding_error_in(meta, "in.RequestString");


  }
};

inline void connect (HelloWorldSupplier& provided, HelloWorldSupplier& required)
{
  provided.out = required.out;
  required.in = provided.in;
  provided.meta.requires = required.meta.requires;
  required.meta.provides = provided.meta.provides;
}





#endif // HELLOWORLDSUPPLIER_HH
