// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef ROBOT_HH
#define ROBOT_HH

#include "Tactics.hh"
#include "Commands.hh"
#include "BallControl.hh"
#include "WheelControl.hh"
#include "PositioningControl.hh"
#include "WheelDriver.hh"
#include "Shooting.hh"
#include "BallHandling.hh"
#include "Navigation.hh"
#include "WorldModel.hh"


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
  WheelDriver wheeldriver;
  Shooting shooting;
  BallHandling ballhandling;
  Navigation navigation;
  WorldModel worldmodel;

  iControl& Control;

  Robot(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;
};

#endif // ROBOT_HH
