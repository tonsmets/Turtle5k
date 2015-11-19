#pragma once

#include "ICompass.h"

class CompassTestStub : public ICompass {
public:
	double getDegrees() override;
};
