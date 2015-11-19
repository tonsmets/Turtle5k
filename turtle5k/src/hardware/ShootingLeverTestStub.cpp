#include "headers/ShootingLeverTestStub.h"

ShootingLeverTestStub::ShootingLeverTestStub()
{
	this->angle = 0;
	this->lastAngleDisplayed = 0;
}

void ShootingLeverTestStub::setAngle(int angle)
{
	this->angle = angle;
}

int ShootingLeverTestStub::getAngle()
{
	if(this->lastAngleDisplayed < this->angle) {
		this->lastAngleDisplayed += this->angle * 0.01;
	} else {
		this->lastAngleDisplayed = this->angle;
	}

	return this->lastAngleDisplayed;
}

void ShootingLeverTestStub::shoot(int meters)
{

}
