#ifndef ROBOT_HH
#define ROBOT_HH

#include "Tactics.hh"
#include "Commands.hh"
#include "BallControl.hh"
#include "DrivingControl.hh"
#include "PositioningControl.hh"
#include "BallHandling.hh"
#include "Shooting.hh"
#include "WheelControl.hh"
#include "WorldModel.hh"
#include "Navigation.hh"


#include "iControl.hh"


namespace dezyne
{
  struct locator;
}


struct Robot
{
  dezyne::meta dzn_meta;
  dezyne::runtime& dzn_rt;
  Tactics tactics;
  Commands commands;
  BallControl ballcontrol;
  DrivingControl drivingControl;
  PositioningControl positioningcontrol;
  BallHandling ballhandling;
  Shooting shooting;
  WheelControl wheelcontrol;
  WorldModel worldmodel;
  Navigation navigation;

  iControl& My_Control;

  Robot(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;
};

#endif // ROBOT_HH
