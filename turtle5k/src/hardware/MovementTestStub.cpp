#include "headers/MovementTestStub.h"
#include <stdlib.h>

MovementTestStub::MovementTestStub()
{
	this->mps = 0;
	this->radians = 0;
}

void MovementTestStub::setSpeed(double mps)
{
	this->mps = mps;
}

void MovementTestStub::setAngle(double radians)
{
	this->radians = radians;
}

double MovementTestStub::getAngle()
{
	return this->radians;
}

double MovementTestStub::getSpeed()
{
	return this->mps * (rand() % 10 / 10);
}
