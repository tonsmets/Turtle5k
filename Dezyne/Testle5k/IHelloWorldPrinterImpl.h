#ifndef IHELLOWORLDPRINTERIMPL_H
#define IHELLOWORLDPRINTERIMPL_H

#include <string>

struct IHelloWorldPrinterImpl
{
	void virtual PrintString(std::string value) = 0;
};

#endif