#ifndef TEST_GAIT_H
#define TEST_GAIT_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include <Robot_Type_III.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct twParam final:public aris::server::GaitParamBase
{
	std::int32_t totalCount{ 9000 };
	double pitchMax{ 0.2 };
	double rollMax{ 0.2 };
	double diameter{ 0.1 };
	double height{ 0.05 };
};

auto twistWaistGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

struct walkParam final :public aris::server::GaitParamBase
{
	std::int32_t totalCount{ 3000 };
	std::int32_t n{ 2 };
	double d{ 0.5 };
	double h{ 0.05 };
	double alpha{ 0.3 };
	double beta{ 0.3 };
};

auto walkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // TEST_GAIT_H
