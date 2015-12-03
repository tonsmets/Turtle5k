#include "Robot.hh"


Robot::Robot(const dezyne::locator& dezyne_locator)
: dzn_meta("","Robot",reinterpret_cast<dezyne::component*>(this),0)
, tactics(dezyne_locator)
, commands(dezyne_locator)
, ballcontrol(dezyne_locator)
, wheelcontrol(dezyne_locator)
, positioningcontrol(dezyne_locator)
, ballhandling(dezyne_locator)
, shooting(dezyne_locator)
, wheeldriver(dezyne_locator)
, worldmodel(dezyne_locator)
, navigation(dezyne_locator)
, My_Control(tactics.My_Control)
{
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&tactics));
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&commands));
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&ballcontrol));
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&wheelcontrol));
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&positioningcontrol));
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&ballhandling));
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&shooting));
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&wheeldriver));
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&worldmodel));
  dzn_meta.children.push_back(reinterpret_cast<dezyne::component*>(&navigation));
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
  ballhandling.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  ballhandling.dzn_meta.address = reinterpret_cast<dezyne::component*>(&ballhandling);
  ballhandling.dzn_meta.name = "ballhandling";
  shooting.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  shooting.dzn_meta.address = reinterpret_cast<dezyne::component*>(&shooting);
  shooting.dzn_meta.name = "shooting";
  wheeldriver.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  wheeldriver.dzn_meta.address = reinterpret_cast<dezyne::component*>(&wheeldriver);
  wheeldriver.dzn_meta.name = "wheeldriver";
  worldmodel.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  worldmodel.dzn_meta.address = reinterpret_cast<dezyne::component*>(&worldmodel);
  worldmodel.dzn_meta.name = "worldmodel";
  navigation.dzn_meta.parent = reinterpret_cast<dezyne::component*>(this);
  navigation.dzn_meta.address = reinterpret_cast<dezyne::component*>(&navigation);
  navigation.dzn_meta.name = "navigation";
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

