#include "creeping_gait.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto creepingParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
	creepingParam param;

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

auto creepingGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotTypeIII &>(model);
	auto &param = static_cast<const creepingParam &>(param_in);

	//模拟地图数据
	double gridMap[120][400]{ 0 };
	for (int i = 0; i < 120; ++i)
	{
		for (int j = 0; j < 150; ++j)
		{
			gridMap[i][j] = -0.895;
		}
		for (int j = 150; j < 400; ++j)
		{
			gridMap[i][j] = -0.895 + 0.01*(j - 150)*std::tan(PI * 35 / 180);
		}
	}

	//初始化
	static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18];
	static double beginHeight;
	if (param.count == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		double begin_pee[18];
		robot.GetPee(begin_pee, beginMak);
		beginHeight = begin_pee[1];
	}
	if (param.count%param.totalCount == 0)
	{
		robot.GetPee(beginPee, beginMak);
	}
	const int LegSeq[3]{ 1, 0, 2 };
	int k = LegSeq[(param.count / param.totalCount) % 3];

	static basicParam basic_param;
	std::fill_n(basic_param.lifting_leg, 6, false);
	basic_param.lifting_leg[k] = true;
	basic_param.lifting_leg[k + 3] = true;
	basic_param.count = param.count % param.totalCount;
	basic_param.beginMak = &beginMak;

	std::copy_n(beginPee, 18, basic_param.targetPee);
	double step_d = param.d;
	if (param.count < param.totalCount)
	{
		step_d = param.d / 2;
	}
	for (int i = 0; i < 2; ++i)
	{
		double targetPee[3];
		std::copy_n(beginPee + 3 * k + 9 * i, 3, targetPee);
		double x0 = targetPee[0];
		double z0 = targetPee[2] - step_d;
		int m = int((0.6 + x0) / 0.01);
		int n = int((0.7 - z0) / 0.01);
		double y0 = gridMap[m][n];
		double dy0 = y0 - targetPee[1];
		double alpha0 = std::atan(dy0 / step_d);
		double d = step_d * std::cos(alpha0);
		double z = targetPee[2] - d;
		n = int((0.7 - z) / 0.01);
		double y = gridMap[m][n] + 0.045;
		basic_param.targetPee[3 * k + 9 * i] = x0;
		basic_param.targetPee[3 * k + 9 * i + 1] = y;
		basic_param.targetPee[3 * k + 9 * i + 2] = z;
		//test
		//if (param.count%param.totalCount == 0)
		//{
		//	rt_printf("x0 = %f\n", x0);
		//	rt_printf("y0 = %f\n", y0);
		//	rt_printf("z0 = %f\n", z0);
		//	rt_printf("alpha0 = %f\n", alpha0);
		//	rt_printf("d = %f\n", d);
		//	rt_printf("m = %d\n", m);
		//	rt_printf("n = %d\n", n);
		//	rt_printf("z = %f\n", z);
		//	rt_printf("y = %f\n\n", y);
		//}
	}

	basic_param.targetPeb[1] = (basic_param.targetPee[1] + basic_param.targetPee[10] + basic_param.targetPee[7] + basic_param.targetPee[16]) / 4 - beginHeight;
	basic_param.targetPeb[2] = (basic_param.targetPee[2] + basic_param.targetPee[11] + basic_param.targetPee[8] + basic_param.targetPee[17]) / 4;
	double dy = ((basic_param.targetPee[1] - basic_param.targetPee[7]) + (basic_param.targetPee[10] - basic_param.targetPee[16])) / 2;
	double dz = -((basic_param.targetPee[2] - basic_param.targetPee[8]) + (basic_param.targetPee[11] - basic_param.targetPee[17])) / 2;
	basic_param.targetWa = std::atan(dy / dz);
	int ret = basicGait(robot, basic_param);
	//for test
	//if (param.count%param.totalCount == 0)
	//{
	//	rt_printf("count: %d\n", param.count);
	//	rt_printf("dy: %f\n", dy);
	//	rt_printf("dz: %f\n", dz);
	//}

	//行程保护
	double pin[18];
	robot.GetPin(pin);
	const double p1_max = 1.067;
	const double p1_min = 0.639;
	const double p23_max = 1.09;
	const double p23_min = 0.69;
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

    return 6 * param.totalCount - param.count - 1;
}

auto basicGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotTypeIII &>(model);
	auto &param = static_cast<const basicParam &>(param_in);

	//初始化
	//static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18]{ 0 };
	static double beginPeb[6]{ 0 };
	static double beginWa{ 0 };
	if (param.count == 0)
	{
		//beginMak.setPrtPm(*robot.body().pm());
		//beginMak.update();
		robot.GetPee(beginPee, *param.beginMak);
		robot.GetPeb(beginPeb, *param.beginMak);
		robot.GetWa(beginWa);

		//for test
		rt_printf("lifting_leg: %d %d %d %d %d %d\n", param.lifting_leg[0], param.lifting_leg[1],
			param.lifting_leg[2], param.lifting_leg[3], param.lifting_leg[4], param.lifting_leg[5]);
		rt_printf("Get beginWa: %f\n", beginWa);
		rt_printf("beginPee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f\n\n", 
			beginPee[0], beginPee[1], beginPee[2], beginPee[3], beginPee[4], beginPee[5], 
			beginPee[6], beginPee[7], beginPee[8], beginPee[9], beginPee[10], beginPee[11], 
			beginPee[12], beginPee[13], beginPee[14], beginPee[15], beginPee[16], beginPee[17]);

		rt_printf("targetWa: %f\n", param.targetWa);
		rt_printf("targetPeb: %f %f %f %f %f %f\n",
			param.targetPeb[0], param.targetPeb[1], param.targetPeb[2], param.targetPeb[3], param.targetPeb[4], param.targetPeb[5]);
		rt_printf("targetPee: \n%f %f %f %f %f %f %f %f %f \n%f %f %f %f %f %f %f %f %f\n\n",
			param.targetPee[0], param.targetPee[1], param.targetPee[2], param.targetPee[3], param.targetPee[4], param.targetPee[5],
			param.targetPee[6], param.targetPee[7], param.targetPee[8], param.targetPee[9], param.targetPee[10], param.targetPee[11],
			param.targetPee[12], param.targetPee[13], param.targetPee[14], param.targetPee[15], param.targetPee[16], param.targetPee[17]);
	}

	double Peb[6]{ 0 }, Pee[18]{ 0 };
	double Wa{ 0 };
	const double s = -0.5 * std::cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1
	//身体插值
	for (int i = 0; i < 3; ++i)
	{
		Peb[i] = param.targetPeb[i] * s + beginPeb[i] * (1 - s);
	}
	//足尖插值
	double h = param.h;
	std::copy(beginPee, beginPee + 18, Pee);
	for (int i = 0; i < 6; ++i)
	{
		if (param.lifting_leg[i])
		{
			double dx = param.targetPee[3 * i] - beginPee[3 * i];
			double dy = param.targetPee[3 * i + 1] - beginPee[3 * i + 1];
			double dz = param.targetPee[3 * i + 2] - beginPee[3 * i + 2];
			double alpha = std::atan(-dy / dz);
			double d = dz / std::cos(alpha);
			double y0 = h * std::sin(PI * s);
			double z0 = d / 2 * (1 - std::cos(PI * s));
			Pee[3 * i] += dx * s;
			Pee[3 * i + 1] += std::cos(alpha) * y0 - std::sin(alpha) * z0;
			Pee[3 * i + 2] += std::sin(alpha) * y0 + std::cos(alpha) * z0;
		}
	}
	//腰关节插值
	Wa = beginWa * (1 - s) + param.targetWa * s;
	//test
	//if (param.count % 100 == 0)
	//{
	//	rt_printf("count: %d\n", param.count);
	//	rt_printf("SetWa: %f\n", Wa);
	//}

	robot.SetPeb(Peb, *param.beginMak);
	robot.SetWa(Wa);
	robot.SetPee(Pee, *param.beginMak);

	return param.totalCount - param.count - 1;
}