#include "Robot.hh"


Robot::Robot(const dezyne::locator& dezyne_locator)
: dzn_meta("","Robot",0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, tactics(dezyne_locator)
, commands(dezyne_locator)
, ballcontrol(dezyne_locator)
, drivingControl(dezyne_locator)
, positioningcontrol(dezyne_locator)
, ballhandling(dezyne_locator)
, shooting(dezyne_locator)
, wheelcontrol(dezyne_locator)
, worldmodel(dezyne_locator)
, navigation(dezyne_locator)
, My_Control(tactics.My_Control)
{
  dzn_meta.children.push_back(&tactics.dzn_meta);
  dzn_meta.children.push_back(&commands.dzn_meta);
  dzn_meta.children.push_back(&ballcontrol.dzn_meta);
  dzn_meta.children.push_back(&drivingControl.dzn_meta);
  dzn_meta.children.push_back(&positioningcontrol.dzn_meta);
  dzn_meta.children.push_back(&ballhandling.dzn_meta);
  dzn_meta.children.push_back(&shooting.dzn_meta);
  dzn_meta.children.push_back(&wheelcontrol.dzn_meta);
  dzn_meta.children.push_back(&worldmodel.dzn_meta);
  dzn_meta.children.push_back(&navigation.dzn_meta);
  tactics.dzn_meta.parent = &dzn_meta;
  tactics.dzn_meta.name = "tactics";
  commands.dzn_meta.parent = &dzn_meta;
  commands.dzn_meta.name = "commands";
  ballcontrol.dzn_meta.parent = &dzn_meta;
  ballcontrol.dzn_meta.name = "ballcontrol";
  drivingControl.dzn_meta.parent = &dzn_meta;
  drivingControl.dzn_meta.name = "drivingControl";
  positioningcontrol.dzn_meta.parent = &dzn_meta;
  positioningcontrol.dzn_meta.name = "positioningcontrol";
  ballhandling.dzn_meta.parent = &dzn_meta;
  ballhandling.dzn_meta.name = "ballhandling";
  shooting.dzn_meta.parent = &dzn_meta;
  shooting.dzn_meta.name = "shooting";
  wheelcontrol.dzn_meta.parent = &dzn_meta;
  wheelcontrol.dzn_meta.name = "wheelcontrol";
  worldmodel.dzn_meta.parent = &dzn_meta;
  worldmodel.dzn_meta.name = "worldmodel";
  navigation.dzn_meta.parent = &dzn_meta;
  navigation.dzn_meta.name = "navigation";
  connect(commands.My_Commands, tactics.My_Commands);
  connect(ballcontrol.My_BallControl, commands.My_BallControl);
  connect(drivingControl.My_DrivingControl, commands.My_DrivingControl);
  connect(positioningcontrol.My_PositioningControl, commands.My_PositioningControl);
  connect(ballhandling.My_BallHandling, ballcontrol.My_BallHandling);
  connect(shooting.My_Shooting, ballcontrol.My_Shooting);
  connect(wheelcontrol.My_WheelControl, drivingControl.My_WheelControl);
  connect(worldmodel.My_WorldModel, positioningcontrol.My_WorldModel);
  connect(navigation.My_Navigation, positioningcontrol.My_Navigation);
}

void Robot::check_bindings() const
{
  dezyne::check_bindings(&dzn_meta);
}
void Robot::dump_tree() const
{
  dezyne::dump_tree(&dzn_meta);
}

