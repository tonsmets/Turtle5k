#pragma once

#include "ICompass.h"

class CompassTestStub : public ICompass {
private:
	double lastDegrees;

public:
	CompassTestStub();
	double getDegrees() override;
};
