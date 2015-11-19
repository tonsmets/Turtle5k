#pragma once

class IMovement {

public:
	virtual ~IMovement();

	virtual void setSpeed(double mps) = 0;
	virtual void setAngle(double radians) = 0;

	virtual double getAngle() = 0;
	virtual double getSpeed() = 0;
};
