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
#ifndef HELLOWORLDAPP_HH
#define HELLOWORLDAPP_HH

#include "meta.hh"

#include <cassert>
#include <map>




struct HelloWorldApp
{

  struct
  {
    std::function<void ()> start;
  } in;

  struct
  {
  } out;

  dezyne::port::meta meta;
  inline HelloWorldApp(dezyne::port::meta m) : meta(m) {}

  void check_bindings() const
  {
    if (! in.start) throw dezyne::binding_error_in(meta, "in.start");


  }
};

inline void connect (HelloWorldApp& provided, HelloWorldApp& required)
{
  provided.out = required.out;
  required.in = provided.in;
  provided.meta.requires = required.meta.requires;
  required.meta.provides = provided.meta.provides;
}





#endif // HELLOWORLDAPP_HH
