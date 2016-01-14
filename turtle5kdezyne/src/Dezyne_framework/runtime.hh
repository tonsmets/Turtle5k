#ifndef RUNTIME_HH
#define RUNTIME_HH

#include "meta.hh"
#include "locator.hh"

#include <boost/tuple/tuple.hpp>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <map>
#include <queue>

namespace dezyne
{
  void trace_in(std::ostream&, port::meta const&, const char*);
  void trace_out(std::ostream&, port::meta const&, const char*);

  inline void apply(const component* t, const boost::function<void(const dezyne::meta&)>& f)
  {
    f(t->dzn_meta);
    for(std::vector<const component*>::const_iterator c = t->dzn_meta.children.begin(); c != t->dzn_meta.children.end(); ++c)
    {
      apply(*c, f);
    }
  }

  inline void check_bindings_helper(const dezyne::meta& m)
  {
    for(std::vector<boost::function<void()> >::const_iterator p = m.ports_connected.begin(); p != m.ports_connected.end(); ++p)
    {
      p->operator()();
    }
  }

  inline void check_bindings(const component* c)
  {
    dezyne::apply(c, check_bindings_helper);
  }

  inline void dump_tree_helper(const dezyne::meta& m)
  {
    std::clog << path(m) << ":" << m.type << std::endl;
  }

  inline void dump_tree(const component* c)
  {
    dezyne::apply(c, dump_tree_helper);
  }

  struct runtime
  {
    std::map<void*, boost::tuple<bool, void*, std::queue<boost::function<void()> >, bool> > queues;

    bool external(void*);
    bool& handling(void*);
    void*& deferred(void*);
    std::queue<boost::function<void()> >& queue(void*);
    bool& performs_flush(void* scope);
    void flush(void*);
    void defer(void*, void*, const boost::function<void()>&);
    void handle(void*, const boost::function<void()>&); // trace_data const&);

    template <typename R>
    R valued_helper(void* scope, const boost::function<R()>& event)
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

  template <typename T>
  boost::reference_wrapper<typename boost::remove_reference<T>::type> ref(T& t)
  {
    return boost::ref(t);
  }
  template <typename T>
  boost::reference_wrapper<typename boost::remove_reference<const T>::type> ref(const T& t)
  {
    return boost::cref(t);
  }
  template <typename C, typename P>
  void call_in(C* c, boost::function<void()> f, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.handle(c, f);
    trace_out(os, boost::get<0>(m)->meta, boost::get<2>(m) ? boost::get<2>(m) : "return");
  }
  template <typename C, typename P, typename A0>
  void call_in(C* c, boost::function<void(A0)> f, A0 a0, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.handle(c, boost::bind(f, ref(a0)));
    trace_out(os, boost::get<0>(m)->meta, boost::get<2>(m) ? boost::get<2>(m) : "return");
  }
  template <typename C, typename P, typename A0, typename A1>
  void call_in(C* c, boost::function<void(A0,A1)> f, A0 a0, A1 a1, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.handle(c, boost::bind(f, ref(a0), ref(a1)));
    trace_out(os, boost::get<0>(m)->meta, boost::get<2>(m) ? boost::get<2>(m) : "return");
  }
  template <typename C, typename P, typename A0, typename A1, typename A2>
  void call_in(C* c, boost::function<void(A0,A1,A2)> f, A0 a0, A1 a1, A2 a2, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.handle(c, boost::bind(f, ref(a0), ref(a1), ref(a2)));
    trace_out(os, boost::get<0>(m)->meta, boost::get<2>(m) ? boost::get<2>(m) : "return");
  }
  template <typename C, typename P, typename A0, typename A1, typename A2, typename A3>
  void call_in(C* c, boost::function<void(A0,A1,A2,A3)> f, A0 a0, A1 a1, A2 a2, A3 a3, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.handle(c, boost::bind(f, ref(a0), ref(a1), ref(a2), ref(a3)));
    trace_out(os, boost::get<0>(m)->meta, boost::get<2>(m) ? boost::get<2>(m) : "return");
  }
  template <typename C, typename P, typename A0, typename A1, typename A2, typename A3, typename A4>
  void call_in(C* c, boost::function<void(A0,A1,A2,A3,A4)> f, A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.handle(c, boost::bind(f, ref(a0), ref(a1), ref(a2), ref(a3), ref(a4)));
    trace_out(os, boost::get<0>(m)->meta, boost::get<2>(m) ? boost::get<2>(m) : "return");
  }
  template <typename C, typename P, typename A0, typename A1, typename A2, typename A3, typename A4, typename A5>
  void call_in(C* c, boost::function<void(A0,A1,A2,A3,A4,A5)> f, A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.handle(c, boost::bind(f, ref(a0), ref(a1), ref(a2), ref(a3), ref(a4), ref(a5)));
    trace_out(os, boost::get<0>(m)->meta, boost::get<2>(m) ? boost::get<2>(m) : "return");
  }


  template <typename R, typename C, typename P>
  R rcall_in(C* c, boost::function<R()> f, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    R r = c->dzn_rt.valued_helper(c, f);
    trace_out(os, boost::get<0>(m)->meta, to_string (r));
    return r;
  }
  template <typename R, typename C, typename P, typename A0>
  R rcall_in(C* c, boost::function<R(A0)> f, A0 a0, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    R r = c->dzn_rt.valued_helper(c, boost::function<R()>(boost::bind(f, ref(a0))));
    trace_out(os, boost::get<0>(m)->meta, to_string (r));
    return r;
  }
  template <typename R, typename C, typename P, typename A0, typename A1>
  R rcall_in(C* c, boost::function<R(A0,A1)> f, A0 a0, A1 a1, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    R r = c->dzn_rt.valued_helper(c, boost::function<R()>(boost::bind(f, ref(a0), ref(a1))));
    trace_out(os, boost::get<0>(m)->meta, to_string (r));
    return r;
  }
  template <typename R, typename C, typename P, typename A0, typename A1, typename A2>
  R rcall_in(C* c, boost::function<R(A0,A1,A2)> f, A0 a0, A1 a1, A2 a2, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    R r = c->dzn_rt.valued_helper(c, boost::function<R()>(boost::bind(f, ref(a0), ref(a1), ref(a2))));
    trace_out(os, boost::get<0>(m)->meta, to_string (r));
    return r;
  }
  template <typename R, typename C, typename P, typename A0, typename A1, typename A2, typename A3>
  R rcall_in(C* c, boost::function<R(A0,A1,A2,A3)> f, A0 a0, A1 a1, A2 a2, A3 a3, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    R r = c->dzn_rt.valued_helper(c, boost::function<R()>(boost::bind(f, ref(a0), ref(a1), ref(a2), ref(a3))));
    trace_out(os, boost::get<0>(m)->meta, to_string (r));
    return r;
  }
  template <typename R, typename C, typename P, typename A0, typename A1, typename A2, typename A3, typename A4>
  R rcall_in(C* c, boost::function<R(A0,A1,A2,A3,A4)> f, A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    R r = c->dzn_rt.valued_helper(c, boost::function<R()>(boost::bind(f, ref(a0), ref(a1), ref(a2), ref(a3), ref(a4))));
    trace_out(os, boost::get<0>(m)->meta, to_string (r));
    return r;
  }
  template <typename R, typename C, typename P, typename A0, typename A1, typename A2, typename A3, typename A4, typename A5>
  R rcall_in(C* c, boost::function<R(A0,A1,A2,A3,A4,A5)> f, A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_in(os, boost::get<0>(m)->meta, boost::get<1>(m));
    R r = c->dzn_rt.valued_helper(c, boost::function<R()>(boost::bind(f, ref(a0), ref(a1), ref(a2), ref(a3), ref(a4), ref(a5))));
    trace_out(os, boost::get<0>(m)->meta, to_string (r));
    return r;
  }


  template <typename C, typename P>
  void call_out(C* c, boost::function<void()> f, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_out(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.defer(boost::get<0>(m)->meta.provides.address, c, f);
  }
  template <typename C, typename P, typename A0>
  void call_out(C* c, boost::function<void(A0)> f, A0 a0, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_out(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.defer(boost::get<0>(m)->meta.provides.address, c, boost::bind(f, a0));
  }
  template <typename C, typename P, typename A0, typename A1>
  void call_out(C* c, boost::function<void(A0,A1)> f, A0 a0, A1 a1, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_out(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.defer(boost::get<0>(m)->meta.provides.address, c, boost::bind(f, a0, a1));
  }
  template <typename C, typename P, typename A0, typename A1, typename A2>
  void call_out(C* c, boost::function<void(A0,A1,A2)> f, A0 a0, A1 a1, A2 a2, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_out(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.defer(boost::get<0>(m)->meta.provides.address, c, boost::bind(f, a0, a1, a2));
  }
  template <typename C, typename P, typename A0, typename A1, typename A2, typename A3>
  void call_out(C* c, boost::function<void(A0,A1,A2,A3)> f, A0 a0, A1 a1, A2 a2, A3 a3, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_out(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.defer(boost::get<0>(m)->meta.provides.address, c, boost::bind(f, a0, a1, a2, a3));
  }
  template <typename C, typename P, typename A0, typename A1, typename A2, typename A3, typename A4>
  void call_out(C* c, boost::function<void(A0,A1,A2,A3,A4)> f, A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_out(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.defer(boost::get<0>(m)->meta.provides.address, c, boost::bind(f, a0, a1, a2, a3, a4));
  }
  template <typename C, typename P, typename A0, typename A1, typename A2, typename A3, typename A4, typename A5>
  void call_out(C* c, boost::function<void(A0,A1,A2,A3,A4,A5)> f, A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, boost::tuple<P*, const char*, const char*> m)
  {
    std::ostream& os = c->dzn_locator.template get<typename std::ostream>();
    trace_out(os, boost::get<0>(m)->meta, boost::get<1>(m));
    c->dzn_rt.defer(boost::get<0>(m)->meta.provides.address, c, boost::bind(f, a0, a1, a2, a3, a4, a5));
  }
}
#endif
