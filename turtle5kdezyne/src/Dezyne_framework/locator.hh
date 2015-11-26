#ifndef LOCATOR_HH
#define LOCATOR_HH

#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <typeinfo>

namespace dezyne {
struct locator
{
private:
  typedef std::string Key;
  struct type_info
  {
    const std::type_info* t;
    type_info(const std::type_info& t)
    : t(&t)
    {}
    bool operator < (const type_info& that) const
    {
      return t->before(*that.t);
    }
  };
  std::map<std::pair<Key,type_info>, void*> services;
public:
  locator()
  {
    set(std::clog);
  }
  locator clone() const
  {
    return locator(*this);
  }
  template <typename T>
  locator& set(T& t, const Key& key = Key())
  {
    services.insert(std::make_pair(std::make_pair(key,type_info(typeid(T))), &t));
    return *this;
  }
  template <typename T>
  T* try_get(const Key& key = Key()) const
  {
    std::map<std::pair<Key,type_info>, void*>::const_iterator it = services.find(std::make_pair(key,type_info(typeid(T))));
    if(it != services.end() && it->second)
      return reinterpret_cast<T*>(it->second);
    return 0;
  }
  template <typename T>
  T& get(const Key& key = Key()) const
  {
    if(T* t = try_get<T>(key))
      return *t;
    throw std::runtime_error("<" + std::string(typeid(T).name()) + ",\"" + key + "\"> not available");
  }
};
}
#endif
