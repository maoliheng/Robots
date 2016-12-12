/* 步态功能：跨沟
* by liujimu, 2016-7-7
*/

/*将以下注释代码添加到xml文件*/
/*
            <cc default="cc_param">
                <cc_param type="group">
				    <totalCount abbreviation="t" type="int" default="2000"/>
				    <chasmWidth abbreviation="d" type="double" default="1"/>
				    <stepHeight abbreviation="h" type="double" default="0.05"/>
                </cc_param>
            </cc>
*/

#ifndef WALK_ON_RAMP
#define WALK_ON_RAMP

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

struct worParam final :public aris::server::GaitParamBase
{
	std::int32_t totalCount{ 2000 };
	double angle{ 35 };
	double d{ 0.3 };
	double h{ 0.05 };
	int n{ 1 };
};

auto walkOnRampParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto walkOnRampGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // WALK_ON_RAMP
