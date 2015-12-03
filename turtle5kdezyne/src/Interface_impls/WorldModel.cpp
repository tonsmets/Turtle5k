#include "WorldModel.hh"

WorldModel::WorldModel(const dezyne::locator& dezyne_locator)
: dzn_meta("","WorldModel",reinterpret_cast<const dezyne::component*>(this),0)
, dzn_rt(dezyne_locator.get<dezyne::runtime>())
, dzn_locator(dezyne_locator)
, My_WorldModel()
{
	dzn_meta.ports_connected.push_back(boost::function<void()>(boost::bind(&iWorldModel::check_bindings,&My_WorldModel)));
	My_WorldModel.meta.provides.port = "WorldModel";
	My_WorldModel.meta.provides.address = this;
	
	dzn_rt.performs_flush(this) = true;
	
	My_WorldModel.in.findTheBall = boost::bind(&dezyne::rcall_in< ::returnResult::type, WorldModel, iWorldModel>,this,boost::function< returnResult::type()>(boost::bind(&WorldModel::My_WorldModel_findTheBall,this)),boost::make_tuple(&My_WorldModel, "findTheBall", "return"));
}

returnResult::type WorldModel::My_WorldModel_findTheBall()
{
	reply__returnResult = returnResult::success;
	return reply__returnResult;
}
