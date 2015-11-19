#pragma once

class IShootingLever {


public:
	virtual ~IShootingLever();
	virtual void setAngle(int angle) = 0;
	virtual int getAngle() = 0;
	virtual void shoot(int meters) = 0;
};
