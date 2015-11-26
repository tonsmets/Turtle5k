#ifndef IWHEELCONTROL_HH
#define IWHEELCONTROL_HH

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


struct iWheelControl
{

  struct
  {
    std::function<returnResult::type ()> getToTheBall;
  } in;

  struct
  {
  } out;

  dezyne::port::meta meta;
  inline iWheelControl(dezyne::port::meta m) : meta(m) {}

  void check_bindings() const
  {
    if (! in.getToTheBall) throw dezyne::binding_error_in(meta, "in.getToTheBall");


  }
};

inline void connect (iWheelControl& provided, iWheelControl& required)
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


#endif // IWHEELCONTROL_HH
