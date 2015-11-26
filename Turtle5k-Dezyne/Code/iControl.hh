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
#ifndef ICONTROL_HH
#define ICONTROL_HH

#include "meta.hh"

#include <cassert>
#include <map>


#ifndef ENUM__returnResult
#define ENUM__returnResult 1
struct returnResult
{
  enum type
  {
    busy, succes, fail, yes, no, stub
  };
};
#endif // ENUM__returnResult


struct iControl
{

  struct
  {
    std::function<returnResult::type ()> tac_getTheBall;
    std::function<returnResult::type ()> tac_shootBall;
    std::function<returnResult::type ()> tac_attack;
  } in;

  struct
  {
  } out;

  dezyne::port::meta meta;
  inline iControl(dezyne::port::meta m) : meta(m) {}

  void check_bindings() const
  {
    if (! in.tac_getTheBall) throw dezyne::binding_error_in(meta, "in.tac_getTheBall");
    if (! in.tac_shootBall) throw dezyne::binding_error_in(meta, "in.tac_shootBall");
    if (! in.tac_attack) throw dezyne::binding_error_in(meta, "in.tac_attack");


  }
};

inline void connect (iControl& provided, iControl& required)
{
  provided.out = required.out;
  required.in = provided.in;
  provided.meta.requires = required.meta.requires;
  required.meta.provides = provided.meta.provides;
}

#ifndef ENUM_TO_STRING__returnResult
#define ENUM_TO_STRING__returnResult 1
inline const char* to_string(::returnResult::type v)
{
  switch(v)
  {
    case ::returnResult::busy: return "returnResult_busy";
    case ::returnResult::succes: return "returnResult_succes";
    case ::returnResult::fail: return "returnResult_fail";
    case ::returnResult::yes: return "returnResult_yes";
    case ::returnResult::no: return "returnResult_no";
    case ::returnResult::stub: return "returnResult_stub";

  }
  return "";
}
#endif // ENUM_TO_STRING__returnResult


#ifndef STRING_TO_ENUM__returnResult
#define STRING_TO_ENUM__returnResult 1
inline ::returnResult::type to__returnResult(std::string s)
{
  static std::map<std::string, ::returnResult::type> m = {
    {"returnResult_busy",::returnResult::busy},
    {"returnResult_succes",::returnResult::succes},
    {"returnResult_fail",::returnResult::fail},
    {"returnResult_yes",::returnResult::yes},
    {"returnResult_no",::returnResult::no},
    {"returnResult_stub",::returnResult::stub},
  };
  if (m.find(s) != m.end())
  {
    return m[s];
  }
  return (::returnResult::type)-1;
}
#endif // STRING_TO_ENUM__returnResult


#endif // ICONTROL_HH
