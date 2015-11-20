#pragma once
#include "IMovement.h"

class MovementTestStub : public IMovement {
private:
	double mps;
	double radians;

public:

	MovementTestStub();

	void setSpeed(double mps) override;
	void setAngle(double radians) override;
	double getAngle() override;
	double getSpeed() override;
};
