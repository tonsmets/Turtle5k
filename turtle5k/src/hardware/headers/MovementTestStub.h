#pragma once
#include "IMovement.h"

class MovementTestStub : public IMovement {
private:
	double mps;
	double degrees;

public:

	MovementTestStub();

	void setSpeed(double mps) override;
	void setAngle(double degrees) override;
	double getAngle() override;
	double getSpeed() override;
};
