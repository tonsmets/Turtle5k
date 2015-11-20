#pragma once
#include "IShootingLever.h"

class ShootingLever : public IShootingLever {

public:

	ShootingLever();

	void setAngle(int angle) override;
	int getAngle() override;
	void shoot(int meters) override;
};
