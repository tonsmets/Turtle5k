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
  struct meta;

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
        , meta()
        {}
        std::string port;
        void* address;
        const dezyne::meta* meta;
      };
      detail provides;
      detail requires;
    };
  }

  struct meta
  {
    meta(const std::string& name, const std::string& type, const meta* parent)
    : name(name)
    , type(type)
    , parent(parent)
    {}
    meta () {}
    std::string name;
    std::string type;
    const meta* parent;
    std::vector<const meta*> children;
    std::vector<boost::function<void()> > ports_connected;
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

  inline std::string path(const meta* m, std::string p = std::string())
  {
    p = p.empty() ? p : "." + p;
    if(!m) return "<external>" + p;
    if(!m->parent) return m->name + p;
    return path(m->parent, m->name + p);
  }

  struct binding_error: public std::runtime_error
  {
    binding_error(const port::meta& m, const std::string& msg)
    : std::runtime_error("not connected: " + path(m.provides.address ? m.provides.meta : m.requires.meta, m.provides.address ? m.provides.port : m.requires.port) + "." + msg)
    {}
  };
}
#endif
