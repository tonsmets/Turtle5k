#pragma once
#include "IShootingLever.h"

class ShootingLeverTestStub : public IShootingLever {
private:
	int angle;

public:

	ShootingLeverTestStub();

	void setAngle(int angle) override;
	int getAngle() override;
	void shoot(int meters) override;
};
