#include "runtime.hh"

#include <algorithm>
#include <iostream>

namespace dezyne {

runtime::runtime(){}

void trace_in(std::ostream& os, port::meta const& m, const char* e)
{
  os << path(m.requires.address, m.requires.port) << "." << e << " -> "
            << path(m.provides.address, m.provides.port) << "." << e << std::endl;
}

void trace_out(std::ostream& os, port::meta const& m, const char* e)
{
  os << path(m.provides.address, m.provides.port) << "." << e << " -> "
     << path(m.requires.address, m.requires.port) << "." << e << std::endl;
}

bool runtime::external(void* scope) {
  return (queues.find(scope) == queues.end());
}

bool& runtime::handling(void* scope)
{
  return std::get<0>(queues[scope]);
}

void*& runtime::deferred(void* scope)
{
  return std::get<1>(queues[scope]);
}

std::queue<std::function<void()> >& runtime::queue(void* scope)
{
  return std::get<2>(queues[scope]);
}

bool& runtime::performs_flush(void* scope)
{
  return std::get<3>(queues[scope]);
}

void runtime::flush(void* scope)
{
#ifdef DEBUG_RUNTIME
  std::cout << path(scope) << " flush" << std::endl;
#endif
  if(!external(scope))
  {
    std::queue<std::function<void()> >& q = queue(scope);
    while(! q.empty())
    {
      std::function<void()> event = q.front();
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

void runtime::defer(void* src, void* tgt, const std::function<void()>& event)
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

void runtime::handle(void* scope, const std::function<void()>& event)
{
  bool& handle = handling(scope);

#ifdef DEBUG_RUNTIME
  std::cout << path(scope) << " handle " << std::boolalpha << handle << std::endl;
#endif

  if(! handle)
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
