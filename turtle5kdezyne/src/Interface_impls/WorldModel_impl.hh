#ifndef WORLDMODEL_IMPL_HH
#define WORLDMODEL_IMPL_HH

#include "iWorldModel.hh"

class WorldModel_impl
{
	public:
	WorldModel_impl();
	static returnResult::type findTheBall(float& x, float& y, float& z);
	static float my_x;
	static float my_y;
	static float my_z;
};

#endif //WORLDMODEL_IMPL_HH
