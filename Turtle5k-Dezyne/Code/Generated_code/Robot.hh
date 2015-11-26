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


#include "iControl.hh"
#include "iBallHandling.hh"
#include "iShooting.hh"
#include "iWheelDriver.hh"
#include "iWorldModel.hh"
#include "iNavigation.hh"


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

  iControl& My_Control;
  iBallHandling& My_BallHandling;
  iShooting& My_Shooting;
  iWheelDriver& My_WheelDriver;
  iWorldModel& My_WorldModel;
  iNavigation& My_Navigation;

  Robot(const dezyne::locator&);
  void check_bindings() const;
  void dump_tree() const;
};

#endif // ROBOT_HH
