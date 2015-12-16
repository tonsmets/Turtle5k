#include "Shooting.hh"

Shooting::Shooting(const dezyne::locator& dezyne_locator)
: dzn_meta("","Shooting",0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_Shooting()
{
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iShooting::check_bindings,&My_Shooting)));
	My_Shooting.meta.provides.port = "My_Shooting";
	My_Shooting.meta.provides.address = this;
	
	dzn_rt.performs_flush(this) = true;
	
	My_Shooting.in.shootTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, Shooting, iShooting>,this,boost::function< returnResult::type()>(boost::bind(&Shooting::My_Shooting_shootTheBall,this)),boost::make_tuple(&My_Shooting, "shootTheBall", "return"));
}

returnResult::type Shooting::My_Shooting_shootTheBall()
{
	reply__returnResult = returnResult::busy;
	return reply__returnResult;
}
