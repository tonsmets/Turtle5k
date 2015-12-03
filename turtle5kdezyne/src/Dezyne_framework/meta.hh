#ifndef META_HH
#define META_HH

#include <boost/bind.hpp>
#include <boost/bind/protect.hpp>
#include <boost/function.hpp>

#include <cassert>
#include <string>
#include <stdexcept>
#include <vector>

namespace dezyne
{
  namespace port
  {
    struct meta
    {
      meta()
      : provides()
      , requires()
      {}
      struct detail
      {
        detail()
        : port()
        , address()
        {}
        std::string port;
        void*       address;
      };
      detail provides;
      detail requires;
    };
  }

  struct component;

  struct meta
  {
    meta(const std::string& name, const std::string& type, const component* address, const component* parent)
    : name(name)
    , type(type)
    , address(address)
    , parent(parent)
    {}
    std::string name;
    std::string type;
    const component* address;
    const component* parent;
    std::vector<const component*> children;
    std::vector<boost::function<void()> > ports_connected;
  };

  struct component
  {
    dezyne::meta dzn_meta;
  };

  struct illegal_handler
  {
    illegal_handler()
    : illegal(boost::bind(&illegal_handler::throw_handler, this))
    {}
    void throw_handler()
    {
      throw std::runtime_error("illegal");
    }
    boost::function<void()> illegal;
  };

  inline std::string path(meta const& m, const std::string& p = "")
  {
    if(m.parent)
      return path(m.parent->dzn_meta, m.name + (p.empty() ? p : "." + p));
    return m.name + (p.empty() ? p : "." + p);
  }

  inline std::string path(void* c, const std::string& p = "")
  {
    if (!c)
      return "<external>." + p;
    return path(reinterpret_cast<const component*>(c)->dzn_meta, p);
  }

  struct binding_error_in: public std::runtime_error
  {
    template <typename T>
    binding_error_in(T const& m, const std::string& msg)
    : std::runtime_error("not connected: " + path(m.provides.address ? m.provides.address : m.requires.address, m.provides.address ? m.provides.port : m.requires.port) + "." + msg)
    {}
  };
  struct binding_error_out: public std::runtime_error
  {
    template <typename T>
    binding_error_out(T const& m, const std::string& msg)
    : std::runtime_error("not connected: " + path(m.requires.address ? m.requires.address : m.provides.address, m.requires.address ? m.requires.port : m.provides.port) + "." + msg)
    {}
  };
}
#endif
