#pragma once
#include "ICompass.h"

class Compass : public ICompass {
public:
	double getDegrees() override;
};
