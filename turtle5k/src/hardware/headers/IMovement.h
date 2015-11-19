#pragma once

class IMovement {

public:
	virtual ~IMovement();

	virtual void setSpeed(double mps) = 0;
	virtual void setAngle(double degrees) = 0;

	virtual double getAngle() = 0;
	virtual double getSeed() = 0;
};
