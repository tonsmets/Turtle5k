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
#include "Robot.hh"


Robot::Robot(const dezyne::locator& dezyne_locator)
: dzn_meta{"","Robot",reinterpret_cast<dezyne::component*>(this),0,{reinterpret_cast<dezyne::component*>(&tactics),reinterpret_cast<dezyne::component*>(&commands),reinterpret_cast<dezyne::component*>(&ballcontrol),reinterpret_cast<dezyne::component*>(&wheelcontrol),reinterpret_cast<dezyne::component*>(&positioningcontrol),reinterpret_cast<dezyne::component*>(&wheeldriver),reinterpret_cast<dezyne::component*>(&shooting),reinterpret_cast<dezyne::component*>(&ballhandling),reinterpret_cast<dezyne::component*>(&navigation),reinterpret_cast<dezyne::component*>(&worldmodel)},{}}
, tactics(dezyne_locator)
, commands(dezyne_locator)
, ballcontrol(dezyne_locator)
, wheelcontrol(dezyne_locator)
, positioningcontrol(dezyne_locator)
, wheeldriver(dezyne_locator)
, shooting(dezyne_locator)
, ballhandling(dezyne_locator)
, navigation(dezyne_locator)
, worldmodel(dezyne_locator)
, My_Control(tactics.My_Control)
{
  tactics.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  tactics.dzn_meta.address = reinterpret_cast<dezyne::component*>(&tactics);
  tactics.dzn_meta.name = "tactics";
  commands.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  commands.dzn_meta.address = reinterpret_cast<dezyne::component*>(&commands);
  commands.dzn_meta.name = "commands";
  ballcontrol.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  ballcontrol.dzn_meta.address = reinterpret_cast<dezyne::component*>(&ballcontrol);
  ballcontrol.dzn_meta.name = "ballcontrol";
  wheelcontrol.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  wheelcontrol.dzn_meta.address = reinterpret_cast<dezyne::component*>(&wheelcontrol);
  wheelcontrol.dzn_meta.name = "wheelcontrol";
  positioningcontrol.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  positioningcontrol.dzn_meta.address = reinterpret_cast<dezyne::component*>(&positioningcontrol);
  positioningcontrol.dzn_meta.name = "positioningcontrol";
  wheeldriver.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  wheeldriver.dzn_meta.address = reinterpret_cast<dezyne::component*>(&wheeldriver);
  wheeldriver.dzn_meta.name = "wheeldriver";
  shooting.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  shooting.dzn_meta.address = reinterpret_cast<dezyne::component*>(&shooting);
  shooting.dzn_meta.name = "shooting";
  ballhandling.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  ballhandling.dzn_meta.address = reinterpret_cast<dezyne::component*>(&ballhandling);
  ballhandling.dzn_meta.name = "ballhandling";
  navigation.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  navigation.dzn_meta.address = reinterpret_cast<dezyne::component*>(&navigation);
  navigation.dzn_meta.name = "navigation";
  worldmodel.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  worldmodel.dzn_meta.address = reinterpret_cast<dezyne::component*>(&worldmodel);
  worldmodel.dzn_meta.name = "worldmodel";
  connect(commands.My_Commands, tactics.My_Commands);
  connect(ballcontrol.My_BallControl, commands.My_BallControl);
  connect(wheelcontrol.My_WheelControl, commands.My_WheelControl);
  connect(positioningcontrol.My_PositioningControl, commands.My_PositioningControl);
  connect(ballhandling.My_BallHandling, ballcontrol.My_BallHandling);
  connect(shooting.My_Shooting, ballcontrol.My_Shooting);
  connect(wheeldriver.My_WheelDriver, wheelcontrol.My_WheelDriver);
  connect(worldmodel.My_WorldModel, positioningcontrol.My_WorldModel);
  connect(navigation.My_Navigation, positioningcontrol.My_Navigation);
}

void Robot::check_bindings() const
{
  dezyne::check_bindings(reinterpret_cast<const dezyne::component*>(this));
}
void Robot::dump_tree() const
{
  dezyne::dump_tree(reinterpret_cast<const dezyne::component*>(this));
}

