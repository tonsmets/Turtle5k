#include "headers/CompassTestStub.h"
#include <stdlib.h>

double CompassTestStub::getDegrees()
{
	return static_cast<double>(rand() % 3600) / 10.0;
}
