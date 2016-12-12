#include "walk_on_ramp.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto walkOnRampParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
	worParam param;

	for (auto &i : params)
	{
		if (i.first == "totalCount")
		{
			param.totalCount = std::stoi(i.second);
		}
		else if (i.first == "d")
		{
			param.d = std::stod(i.second);
		}
		else if (i.first == "h")
		{
			param.h = std::stod(i.second);
		}
	}

	msg.copyStruct(param);
}

auto walkOnRampGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotTypeIII &>(model);
	auto &param = static_cast<const worParam &>(param_in);

	//初始化
	static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18];
	static double beginWa{ 0 };

	if (param.count%param.totalCount == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		robot.GetWa(beginWa);
		robot.GetPee(beginPee, beginMak);
	}

	int period_count = param.count%param.totalCount;
	const double s = -(PI / 2)*cos(PI * (period_count + 1) / param.totalCount) + PI / 2;//s 从0到PI. 
	const int leg_begin_id = (param.count / param.totalCount) % 2 == 1 ? 3 : 0;

	const double a = PI / 180 * param.angle;
	const double h = param.h;
	double d = param.d;
	double db;

	double Peb[6], Pee[18];
	std::fill(Peb, Peb + 6, 0);
	std::copy(beginPee, beginPee + 18, Pee);

	if ((param.count / param.totalCount) == 0 )//加速段
	{
		d = d / 2;
		db = aris::dynamic::s_interp(param.totalCount, period_count + 1, 0, d / 4, 0, d / 2 / param.totalCount);
	}
	else if ((param.count / param.totalCount) == (param.n * 2 - 1))//减速段
	{
		d = d / 2;
		db = aris::dynamic::s_interp(param.totalCount, period_count + 1, 0, d / 4, d / 2 / param.totalCount, 0);
	}
	else//匀速段
	{
		db = d / 2 * aris::dynamic::even(param.totalCount, period_count + 1);
	}
	//规划腿
	for (int i = leg_begin_id; i < 18; i += 6)
	{
		double dy = h * std::sin(s);
		double dz = d * (1 - std::cos(s)) / 2;
		Pee[i + 1] = beginPee[i + 1] + std::cos(a) * dy + std::sin(a) * dz;
		Pee[i + 2] = beginPee[i + 2] + std::sin(a) * dy - std::cos(a) * dz;
	}
	//规划身体
	Peb[1] = db*std::sin(a);
	Peb[2] = -db*std::cos(a);

	robot.SetPeb(Peb, beginMak);
	robot.SetWa(beginWa);
	robot.SetPee(Pee, beginMak);

	//行程保护
	double pin[18];
	robot.GetPin(pin);
	const double p1_max = 1.112;
	const double p1_min = 0.722;
	const double p23_max = 1.128;
	const double p23_min = 0.738;
	bool out_of_limit = false;
	for (int i = 0; i < 18; ++i)
	{
		double max, min;
		if (i % 3 == 0)
		{
			max = p1_max;
			min = p1_min;
		}
		else
		{
			max = p23_max;
			min = p23_min;
		}
		if (pin[i] > max)
		{
			rt_printf("pin[%d] = %f, longer than limit.\n", i, pin[i]);
			out_of_limit = true;
		}
		else if (pin[i] < min)
		{
			rt_printf("pin[%d] = %f, shorter than limit.\n", i, pin[i]);
			out_of_limit = true;
		}
	}
	if (out_of_limit)
	{
		rt_printf("Pin out of limit at count: %d\n", param.count);
		return 0;
	}

	return 2 * param.n * param.totalCount - param.count - 1;
}
