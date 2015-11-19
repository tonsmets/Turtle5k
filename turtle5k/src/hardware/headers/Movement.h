#pragma once

#include "IMovement.h"

class Movement : public IMovement {
private:
	double mps;
	double degrees;

public:
	void setSpeed(double mps) override;
	void setAngle(double degrees) override;
	double getAngle() override;
	double getSeed() override;
};
