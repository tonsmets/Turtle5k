#include "Navigation.hh"

Navigation::Navigation(const dezyne::locator& dezyne_locator)
: dzn_meta("","Navigation",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_Navigation()
{
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iNavigation::check_bindings,&My_Navigation)));
	My_Navigation.meta.provides.port = "My_Navigation";
	My_Navigation.meta.provides.address = this;
	
	dzn_rt.performs_flush(this) = true;
	
	My_Navigation.in.Navigate = boost::bind(&dezyne::rcall_in< ::returnResult::type, Navigation, iNavigation>,this,boost::function< returnResult::type()>(boost::bind(&Navigation::My_Navigation_Navigate,this)),boost::make_tuple(&My_Navigation, "Navigate", "return"));
}

returnResult::type Navigation::My_Navigation_Navigate()
{
	reply__returnResult = returnResult::fail;
	return reply__returnResult;
}
