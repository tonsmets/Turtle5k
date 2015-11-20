#pragma once

#include "IBallHandling.h"

class BallHandlingTestStub : public IBallHandling {

private:
	double rps;
	double lastRpsSend;

public:

	BallHandlingTestStub();

	void setRotationSpeed(double rps) override;
	double getRotationSpeed() override;
};
