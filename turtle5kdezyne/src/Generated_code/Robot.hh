#ifndef ROBOT_HH
#define ROBOT_HH

#include "Tactics.hh"
#include "Commands.hh"
#include "BallControl.hh"
#include "WheelControl.hh"
#include "PositioningControl.hh"
#include "BallHandling.hh"
#include "Shooting.hh"
#include "WheelDriver.hh"
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
  Tactics tactics;
  Commands commands;
  BallControl ballcontrol;
  WheelControl wheelcontrol;
  PositioningControl positioningcontrol;
  BallHandling ballhandling;
  Shooting shooting;
  WheelDriver wheeldriver;
  WorldModel worldmodel;
  Navigation navigation;

  iControl& My_Control;

  Robot(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;
};

#endif // ROBOT_HH
