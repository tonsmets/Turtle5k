#include "runtime.hh"

#include <algorithm>
#include <iostream>

namespace dezyne {

runtime::runtime(){}

void trace_in(std::ostream& os, port::meta const& m, const char* e)
{
  os << path(m.requires.meta, m.requires.port) << "." << e << " -> "
     << path(m.provides.meta, m.provides.port) << "." << e << std::endl;
}

void trace_out(std::ostream& os, port::meta const& m, const char* e)
{
  os << path(m.provides.meta, m.provides.port) << "." << e << " -> "
     << path(m.requires.meta, m.requires.port) << "." << e << std::endl;
}

bool runtime::external(void* scope) {
  return (queues.find(scope) == queues.end());
}

bool& runtime::handling(void* scope)
{
  return boost::get<0>(queues[scope]);
}

void*& runtime::deferred(void* scope)
{
  return boost::get<1>(queues[scope]);
}

std::queue<boost::function<void()> >& runtime::queue(void* scope)
{
  return boost::get<2>(queues[scope]);
}

bool& runtime::performs_flush(void* scope)
{
  return boost::get<3>(queues[scope]);
}

void runtime::flush(void* scope)
{
#ifdef DEBUG_RUNTIME
  std::cout << path(scope) << " flush" << std::endl;
#endif
  if(!external(scope))
  {
    std::queue<boost::function<void()> >& q = queue(scope);
    while(not q.empty())
    {
      boost::function<void()> event = q.front();
      q.pop();
      handle(scope, event);
    }
    if (deferred(scope)) {
      void* tgt = deferred(scope);
      deferred(scope) = NULL;
      if (!handling(tgt)) {
        runtime::flush(tgt);
      }
    }
  }
}

void runtime::defer(void* src, void* tgt, const boost::function<void()>& event)
{
#ifdef DEBUG_RUNTIME
  std::cout << path(tgt) << " defer" << std::endl;
#endif

  if(!(src && performs_flush(src)) && !handling(tgt))
  {
    handle(tgt, event);
  }
  else
  {
    deferred(src) = tgt;
    queue(tgt).push(event);
  }
}

void runtime::handle(void* scope, const boost::function<void()>& event)
{
  bool& handle = handling(scope);

#ifdef DEBUG_RUNTIME
  std::cout << path(scope) << " handle " << std::boolalpha << handle << std::endl;
#endif

  if(not handle)
  {
    {
      scoped_value<bool> sv(handle, true);
      event();
    }
    flush(scope);
  }
  else
  {
    throw std::logic_error("component already handling an event");
  }
}
}
