#include "headers/MovementTestStub.h"
#include <stdlib.h>

MovementTestStub::MovementTestStub()
{
	this->mps = 0;
	this->degrees = 0;
}

void MovementTestStub::setSpeed(double mps)
{
	this->mps = mps;
}

void MovementTestStub::setAngle(double degrees)
{
	this->degrees = degrees;
}

double MovementTestStub::getAngle()
{
	return this->degrees;
}

double MovementTestStub::getSeed()
{
	return this->mps * (rand() % 10 / 10);
}
