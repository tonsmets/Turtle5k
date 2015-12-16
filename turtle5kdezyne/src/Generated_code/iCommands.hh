#ifndef ICOMMANDS_HH
#define ICOMMANDS_HH

#include "meta.hh"

#include <cassert>
#include <map>


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


struct iCommands
{

  struct
  {
    boost::function<returnResult::type ()> findTheBall;
    boost::function<returnResult::type ()> getToTheBall;
    boost::function<returnResult::type ()> shootTheBall;
    boost::function<returnResult::type ()> getCurrentLocation;
    boost::function<returnResult::type ()> driveToLocation;
    boost::function<returnResult::type ()> driveToTheBall;
  } in;

  struct
  {
  } out;

  dezyne::port::meta meta;

  void check_bindings() const
  {
    if (! in.findTheBall) throw dezyne::binding_error(meta, "in.findTheBall");
    if (! in.getToTheBall) throw dezyne::binding_error(meta, "in.getToTheBall");
    if (! in.shootTheBall) throw dezyne::binding_error(meta, "in.shootTheBall");
    if (! in.getCurrentLocation) throw dezyne::binding_error(meta, "in.getCurrentLocation");
    if (! in.driveToLocation) throw dezyne::binding_error(meta, "in.driveToLocation");
    if (! in.driveToTheBall) throw dezyne::binding_error(meta, "in.driveToTheBall");


  }
};

inline void connect (iCommands& provided, iCommands& required)
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
    case ::returnResult::success: return "returnResult_success";
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
  static std::map<std::string, ::returnResult::type> m;
  if(m.empty())
  {
    m["returnResult_busy"] = ::returnResult::busy;
    m["returnResult_success"] = ::returnResult::success;
    m["returnResult_fail"] = ::returnResult::fail;
    m["returnResult_yes"] = ::returnResult::yes;
    m["returnResult_no"] = ::returnResult::no;
    m["returnResult_stub"] = ::returnResult::stub;

  }
  if (m.find(s) != m.end())
  {
    return m[s];
  }
  return (::returnResult::type)-1;
}
#endif // STRING_TO_ENUM__returnResult


#endif // ICOMMANDS_HH
