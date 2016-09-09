#ifndef MOVE_SINGLE_LEG_H
#define MOVE_SINGLE_LEG_H

#define CROSS_CHASM_H

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
#include <Robot_Type_III.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct mslParam final :public aris::server::GaitParamBase
{
	std::int32_t totalCount{ 3000 };
	int legID{ 0 };
	double displacement[3]{ 0 };
	bool isAbsolute{ false }; //用于判断移动命令是绝对坐标还是相对坐标
};

auto moveSingleLegParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto moveSingleLegGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // MOVE_SINGLE_LEG_H
