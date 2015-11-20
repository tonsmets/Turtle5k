#include "HelloWorldPrinter.h"

#include "locator.hh"
#include "runtime.hh"
#include "meta.hh"

#include "IHelloWorldPrinterImpl.h"

#include <functional>
#include <memory>

#include <string>

namespace dezyne 
{
	HelloWorldPrinter::HelloWorldPrinter(const locator& l)
		: rt(l.get<runtime>())
		, port(l.get<dezyne::port::meta>())
	{
		locator tmp(l.clone());
		tmp.set(port);
		auto pimpl = l.get<std::function<std::shared_ptr<IHelloWorldPrinterImpl>(const locator&)>>()(tmp);
		port.in.PrintString = [=] (std::string value) {pimpl->PrintString(value); };
	}
}
