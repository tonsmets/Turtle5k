#ifndef RUNTIME_HH
#define RUNTIME_HH

#include "meta.hh"
#include "locator.hh"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <map>
#include <queue>
#include <tuple>

namespace dezyne
{
  void trace_in(std::ostream&, port::meta const&, const char*);
  void trace_out(std::ostream&, port::meta const&, const char*);

  inline void apply(const component* t, const std::function<void(const dezyne::meta&)>& f)
  {
    f(t->dzn_meta);
    for (auto c : t->dzn_meta.children)
    {
      apply(c, f);
    }
  }

  inline void check_bindings(const component* c)
  {
    dezyne::apply(c, [](const dezyne::meta& m){
        std::for_each(m.ports_connected.begin(), m.ports_connected.end(), [&](std::function<void()> p){p();});
      });
  }

  inline void dump_tree(const component* c)
  {
    dezyne::apply(c, [](const dezyne::meta& m){
        std::clog << path(m) << ":" << m.type << std::endl;
    });
  }

  struct runtime
  {
    std::map<void*, std::tuple<bool, void*, std::queue<std::function<void()> >, bool> > queues;

    bool external(void*);
    bool& handling(void*);
    void*& deferred(void*);
    std::queue<std::function<void()> >& queue(void*);
    bool& performs_flush(void* scope);
    void flush(void*);
    void defer(void*, void*, const std::function<void()>&);
    void handle(void*, const std::function<void()>&); // trace_data const&);

    template <typename R>
    inline R valued_helper(void* scope, const std::function<R()>& event)
    {
      bool& handle = handling(scope);
      if(handle) throw std::logic_error("a valued event cannot be deferred");
      R tmp;
      {
        runtime::scoped_value<bool> sv(handle, true);
        tmp = event();
      }
      flush(scope);
      return tmp;
    }

    template <typename T>
    struct scoped_value
    {
      T& current;
      T initial;
      scoped_value(T& current, T value)
      : current(current)
      , initial(current)
      { current = value; }
      ~scoped_value()
      {
        current = initial;
      }
    };
    runtime();
  private:
    runtime(const runtime&);
    runtime& operator = (const runtime&);
  };

  template <typename C, typename P>
  void call_in(C* c, std::function<void()> f, std::tuple<P*, const char*, const char*> m)
  {
    auto& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, std::get<0>(m)->meta, std::get<1>(m));
    c->dzn_rt.handle(c, f);
    trace_out(os, std::get<0>(m)->meta, std::get<2>(m) ? std::get<2>(m) : "return");
  }

  template <typename R, typename C, typename P>
  R call_in(C* c, std::function<R()> f, std::tuple<P*, const char*, const char*> m)
  {
    auto& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, std::get<0>(m)->meta, std::get<1>(m));
    auto r = c->dzn_rt.valued_helper(c, f);
    trace_out(os, std::get<0>(m)->meta, to_string (r));
    return r;
  }

  template <typename C, typename P>
  void call_out(C* c, std::function<void()> f, std::tuple<P*, const char*, const char*> m)
  {
    auto& os = c->dzn_locator.template get<typename std::ostream>();
    trace_out(os, std::get<0>(m)->meta, std::get<1>(m));
    c->dzn_rt.defer(std::get<0>(m)->meta.provides.address, c, f);
  }
}
#endif
