#pragma once

class ICompass {

public:
	virtual ~ICompass();

	virtual double getDegrees() = 0;
};
