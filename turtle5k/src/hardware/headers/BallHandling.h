#pragma once

#include "IBallHandling.h"

class BallHandling : public IBallHandling {

private:
	double rps;

public:
	void setRotationSpeed(double rps) override;
	double getRotationSpeed() override;
};
