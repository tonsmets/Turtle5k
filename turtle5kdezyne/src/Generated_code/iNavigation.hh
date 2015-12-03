#ifndef INAVIGATION_HH
#define INAVIGATION_HH

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


struct iNavigation
{

  struct
  {
    boost::function<returnResult::type ()> Navigate;
    boost::function<returnResult::type ()> getCurrentLocation;
  } in;

  struct
  {
  } out;

  dezyne::port::meta meta;

  void check_bindings() const
  {
    if (not in.Navigate) throw dezyne::binding_error_in(meta, "in.Navigate");
    if (not in.getCurrentLocation) throw dezyne::binding_error_in(meta, "in.getCurrentLocation");


  }
};

inline void connect (iNavigation& provided, iNavigation& required)
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


#endif // INAVIGATION_HH
