#include <stdlib.h>

#include "headers/BallHandlingTestStub.h"

BallHandlingTestStub::BallHandlingTestStub()
{
	this->rps = 0;
}

void BallHandlingTestStub::setRotationSpeed(double rps)
{
	this->rps = rps;
}

double BallHandlingTestStub::getRotationSpeed()
{
	return ((double)(rand() % 100) / 100.0) * this->rps;
}
