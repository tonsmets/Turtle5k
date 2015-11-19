#include "headers/CompassTestStub.h"
#include <stdlib.h>

CompassTestStub::CompassTestStub() {
	this->lastDegrees = 0;
}

double CompassTestStub::getDegrees()
{
	this->lastDegrees += static_cast<double>(rand() % 3600) / 1000.0;
	if(this->lastDegrees > 360)
		this->lastDegrees = 0;
	return this->lastDegrees ;
}
