#pragma once

class IBallHandling {

public:
	virtual ~IBallHandling();

	virtual void setRotationSpeed(double rps) = 0;
	virtual double getRotationSpeed() = 0;
};
