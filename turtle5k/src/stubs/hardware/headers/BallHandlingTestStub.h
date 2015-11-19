#pragma once

#include "IBallHandling.h"

class BallHandlingTestStub : public IBallHandling {
private:
	double rps;

public:

	BallHandlingTestStub();

	void setRotationSpeed(double rps) override;
	double getRotationSpeed() override;
};
