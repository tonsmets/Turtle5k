#include "HelloWorldApplication.hh"
#include "IHelloWorldPrinterImpl.h"
#include "HelloWorldPrinter.hh"
#include "locator.hh"
#include "runtime.hh"
#include <map>
#include <csignal>
#include <functional>
#include <memory>
#include <iostream>
#include <string>

struct HelloWorldPrinterImpl : public IHelloWorldPrinterImpl
{
	size_t id;
	IHelloWorldPrinter& port;

	HelloWorldPrinterImpl(const dezyne::locator& l)
		: id(0)
		, port(l.get<IHelloWorldPrinter>())
	{}
	void PrintString(std::string& value)
	{
		std::cout << value << std::endl;
	}
};

void HelloSupplyString(std::string& value) {
	value = "HelloWorld";
}

void HelloPrintString(std::string& value) {
	std::cout << value << std::endl;
}

int main() {
	dezyne::locator locator;
	dezyne::runtime runtime;
	dezyne::port::meta meta;
	locator.set(runtime);
	locator.set(meta);


	std::function<std::shared_ptr<IHelloWorldPrinterImpl>(const dezyne::locator&)> print_string_impl = [](const dezyne::locator& l) {return std::make_shared<HelloWorldPrinterImpl>(l); };
	locator.set(print_string_impl);
	//std::function<std::shared_ptr<helloworldprinterimpl>(const dezyne::locator&)> B = [](const dezyne::locator& l) {return std::make_shared<A>(l); };
	//locator.set(B);


	HelloWorldApplication app(locator);

	app.dzn_meta.name = "app";
	app.supplier.in.RequestString = HelloSupplyString;
	//app.printer.in.PrintString = HelloPrintString;


	app.check_bindings();
	app.app.in.start();


	int dontclosewindow;
	std::cin >> dontclosewindow;

    return 0;
}