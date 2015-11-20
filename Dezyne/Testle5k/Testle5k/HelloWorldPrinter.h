#pragma once

#include "IHelloWorldPrinter.hh"

#include "runtime.hh"
#include <string>

namespace dezyne 
{
	struct locator;
	struct runtime;

	struct HelloWorldPrinter
	{
		dezyne::meta meta;
		runtime& rt;
		IHelloWorldPrinter port;

		HelloWorldPrinter(const locator&);

		private:
			void port_Print(std::string& value);
	};
}