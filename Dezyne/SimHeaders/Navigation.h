#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include "INavigation.hh"

void setCurrentLocation(Odometry location);
void setCurrentDesiredLocation(Odometry location);
double getPidValueSpeed();
double getPidValueAngle(double& angle);

#endif