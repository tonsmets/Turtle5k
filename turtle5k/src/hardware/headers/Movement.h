#pragma once

#include "IMovement.h"

class Movement : public IMovement {
private:
	double mps;
	double radians;

public:
	void setSpeed(double mps) override;
	void setAngle(double radians) override;
	double getAngle() override;
	double getSpeed() override;
};
