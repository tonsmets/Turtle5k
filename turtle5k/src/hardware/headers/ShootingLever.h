#pragma once
#include "IShootingLever.h"

class ShootingLever : public IShootingLever {
private:
	int angle;
	int meters;

public:
	void setAngle(int angle) override;
	int getAngle() override;
	void shoot(int meters) override;
};
