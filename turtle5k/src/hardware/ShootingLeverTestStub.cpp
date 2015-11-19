#include "headers/ShootingLeverTestStub.h"

ShootingLeverTestStub::ShootingLeverTestStub()
{
	this->angle = 0;
}

void ShootingLeverTestStub::setAngle(int angle)
{
	this->angle = angle;
}

int ShootingLeverTestStub::getAngle()
{
	return this->angle;
}

void ShootingLeverTestStub::shoot(int meters)
{

}
