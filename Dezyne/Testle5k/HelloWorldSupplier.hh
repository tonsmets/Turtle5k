// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
