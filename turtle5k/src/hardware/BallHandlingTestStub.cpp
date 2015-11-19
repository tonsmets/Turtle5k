#include <stdlib.h>

#include "headers/BallHandlingTestStub.h"

BallHandlingTestStub::BallHandlingTestStub()
{
	this->rps = 0;
	this->lastRpsSend = 0;
}

void BallHandlingTestStub::setRotationSpeed(double rps)
{
	this->rps = rps;
}

double BallHandlingTestStub::getRotationSpeed()
{
	this->lastRpsSend += (this->lastRpsSend > this->rps ? -(this->rps * 0.01) : (this->rps * 0.01));
	return this->lastRpsSend;
}
