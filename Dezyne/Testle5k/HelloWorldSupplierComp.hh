#ifndef HELLOWORLDSUPPLIERCOMP_HH
#define HELLOWORLDSUPPLIERCOMP_HH

#include "HelloWorldSupplier.hh"


#include "runtime.hh"

namespace dezyne {
  struct locator;
  struct runtime;
}


struct HelloWorldSupplierComp
{
  dezyne::meta dzn_meta;
  dezyne::runtime& dzn_rt;
  dezyne::locator const& dzn_locator;
  HelloWorldSupplier supplier;

  HelloWorldSupplierComp(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;

  private:
  void supplier_RequestString(std::string& value);
};

#endif // HELLOWORLDSUPPLIERCOMP_HH
