#include "test_gait.h"

#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

auto twistWaistGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
    auto &robot = static_cast<Robots::RobotTypeIII &>(model);
    auto &param = static_cast<const twParam &>(param_in);

    //初始化
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static double beginPee[18];
	int totalCount = param.totalCount;
	int prepCount = param.totalCount / 6;
	int twistCount = param.totalCount * 4 / 6;

    if (param.count == 0)
    {
        beginMak.setPrtPm(*robot.body().pm());
        beginMak.update();
        robot.GetPee(beginPee, beginMak);
    }

	double Wa{ 0 };
    double Peb[6], Pee[18];
    std::fill(Peb, Peb + 6, 0);
    std::copy(beginPee, beginPee + 18, Pee);

	double targetPeb[6]{ 0 };
	std::fill(targetPeb, targetPeb + 6, 0);
	targetPeb[1] = param.height;
	targetPeb[2] = -param.diameter / 2;
	targetPeb[4] = param.pitchMax;

	if (param.count < prepCount)
	{
		const double s = -0.5 * cos(PI * (param.count + 1) / prepCount) + 0.5; //s从0到1.
		for (int i = 0; i < 6; ++i)
		{
			Peb[i] = targetPeb[i] * s;
		}
		Wa = PI / 12 * std::sin(s * PI / 2);
	}
	else if (param.count < (prepCount + twistCount))
	{
		const double s = -PI * cos(PI * (param.count + 1 - prepCount) / twistCount) + PI; //s从0到2*PI.
		Peb[0] = param.diameter / 2 * sin(s);
		Peb[1] = targetPeb[1];
		Peb[2] = -param.diameter / 2 * cos(s);
		Peb[3] = param.rollMax * sin(s);
		Peb[4] = param.pitchMax * cos(s);
		Wa = PI / 12 * std::sin(PI / 2);
	}
	else
	{
		const double s = -0.5 * cos(PI * (param.count + 1 - prepCount - twistCount) / (totalCount - prepCount - twistCount)) + 0.5; //s从0到1.
		for (int i = 0; i < 6; ++i)
		{
			Peb[i] = targetPeb[i] * (1 - s);
		}
		Wa = PI / 12 * std::sin((1 - s) * PI / 2);
	}

    robot.SetPeb(Peb, beginMak);
	robot.SetWa(Wa);
    robot.SetPee(Pee, beginMak);

    return param.totalCount - param.count - 1;
}

auto walkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotTypeIII &>(model);
	auto &param = static_cast<const walkParam &>(param_in);

	//初始化
	static aris::dynamic::FloatMarker beginMak{ robot.ground() };
	static double beginPee[18];

	if (param.count%param.totalCount == 0)
	{
		beginMak.setPrtPm(*robot.body().pm());
		beginMak.update();
		robot.GetPee(beginPee, beginMak);
	}

	//以下设置各个阶段的身体的真实初始位置
	const double a = param.alpha;
	const double b = param.beta;
	const double d = param.d;
	const double h = param.h;

	const double front[3]{ -std::sin(a),0,-std::cos(a) };
	const double left[3]{ -std::cos(a),0,std::sin(a) };
	const double up[3]{ 0,1,0 };

	int period_count = param.count%param.totalCount;
	const double s = -(PI / 2)*cos(PI * (period_count + 1) / param.totalCount) + PI / 2;//s 从0到PI. 

	double Peb[6], Pee[18];
	std::fill(Peb, Peb + 6, 0);
	std::copy(beginPee, beginPee + 18, Pee);


	double pq_b[7]{ 0,0,0,std::sin(b / 2)*up[0],std::sin(b / 2)*up[1],std::sin(b / 2)*up[2],std::cos(b / 2) };
	double pq_b_half[7]{ 0,0,0,std::sin(b / 4)*up[0],std::sin(b / 4)*up[1],std::sin(b / 4)*up[2],std::cos(b / 4) };
	double pq_b_quad[7]{ 0,0,0,std::sin(b / 8)*up[0],std::sin(b / 8)*up[1],std::sin(b / 8)*up[2],std::cos(b / 8) };
	double pq_b_eighth[7]{ 0,0,0,std::sin(b / 16)*up[0],std::sin(b / 16)*up[1],std::sin(b / 16)*up[2],std::cos(b / 16) };
	double pm_b[16], pm_b_half[16], pm_b_quad[16], pm_b_eighth[16];

	aris::dynamic::s_pq2pm(pq_b, pm_b);
	aris::dynamic::s_pq2pm(pq_b_half, pm_b_half);
	aris::dynamic::s_pq2pm(pq_b_quad, pm_b_quad);
	aris::dynamic::s_pq2pm(pq_b_eighth, pm_b_eighth);

	const int leg_begin_id = (param.count / param.totalCount) % 2 == 1 ? 3 : 0;

	if ((param.count / param.totalCount) == 0)//加速段
	{
		//规划腿
		for (int i = leg_begin_id; i < 18; i += 6)
		{
			//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
			double leg_forward_dir[3], forward_d[3];
			aris::dynamic::s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

			aris::dynamic::s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
			aris::dynamic::s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
			aris::dynamic::s_daxpy(3, d / 2, leg_forward_dir, 1, forward_d, 1);

			for (int j = 0; j < 3; ++j)
			{
				Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
			}
		}

		//规划身体位置
		double body_forward_dir[3], body_left_dir[3];
		aris::dynamic::s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
		aris::dynamic::s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

		for (int i = 0; i < 3; ++i)
		{
			Peb[i] = left[i] * aris::dynamic::s_interp(param.totalCount, period_count + 1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::sin(b / 4))
				+ front[i] * aris::dynamic::s_interp(param.totalCount, period_count + 1, 0, d / 4 / std::cos(b / 4), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b / 4));
		}

		//规划身体姿态
		double s_acc = aris::dynamic::acc_even(param.totalCount, period_count + 1);
		double pq[7] = { 0,0,0,std::sin(s_acc*b / 8)*up[0],std::sin(s_acc*b / 8)*up[1] ,std::sin(s_acc*b / 8)*up[2],std::cos(s_acc*b / 8) };
		double pe[6];
		aris::dynamic::s_pq2pe(pq, pe);
		std::copy(pe + 3, pe + 6, Peb + 3);
	}
	else if ((param.count / param.totalCount) == (param.n * 2 - 1))//减速段
	{
		//规划腿
		for (int i = leg_begin_id; i < 18; i += 6)
		{
			//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
			double leg_forward_dir[3], forward_d[3];
			aris::dynamic::s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

			aris::dynamic::s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
			aris::dynamic::s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
			aris::dynamic::s_daxpy(3, d / 2, leg_forward_dir, 1, forward_d, 1);

			for (int j = 0; j < 3; ++j)
			{
				Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
			}
		}

		//规划身体位置
		double body_forward_dir[3], body_left_dir[3];
		aris::dynamic::s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
		aris::dynamic::s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

		for (int i = 0; i < 3; ++i)
		{
			Peb[i] = left[i] * aris::dynamic::s_interp(param.totalCount, period_count + 1, 0, d*std::tan(b / 8) / 4 / std::cos(b / 8), 0, 0)
				+ front[i] * aris::dynamic::s_interp(param.totalCount, period_count + 1, 0, d / 4 / std::cos(b / 4), d / 2 / param.totalCount / std::cos(b / 2), 0);
		}

		//规划身体姿态
		double s_dec = aris::dynamic::dec_even(param.totalCount, period_count + 1);
		double pq[7] = { 0,0,0,std::sin(s_dec*b / 8)*up[0],std::sin(s_dec*b / 8)*up[1] ,std::sin(s_dec*b / 8)*up[2],std::cos(s_dec*b / 8) };
		double pe[6];
		aris::dynamic::s_pq2pe(pq, pe);
		std::copy(pe + 3, pe + 6, Peb + 3);
	}
	else//匀速段
	{
		//规划腿
		for (int i = leg_begin_id; i < 18; i += 6)
		{
			//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
			double leg_forward_dir[3], forward_d[3];
			aris::dynamic::s_pm_dot_v3(pm_b_half, front, leg_forward_dir);

			aris::dynamic::s_pm_dot_v3(pm_b, beginPee + i, forward_d);
			aris::dynamic::s_daxpy(3, -1, beginPee + i, 1, forward_d, 1);
			aris::dynamic::s_daxpy(3, d, leg_forward_dir, 1, forward_d, 1);

			for (int j = 0; j < 3; ++j)
			{
				Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
			}
		}

		//规划身体位置
		double d2 = d / 2 / std::cos(b / 4);
		for (int i = 0; i < 3; ++i)
		{
			Peb[i] = left[i] * aris::dynamic::s_interp(param.totalCount, period_count + 1, 0, d2*std::sin(b / 4), 0, d / 2 / param.totalCount / std::cos(b / 2)*std::sin(b / 2))
				+ front[i] * aris::dynamic::s_interp(param.totalCount, period_count + 1, 0, d / 2, d / 2 / param.totalCount / std::cos(b / 2), d / 2 / param.totalCount / std::cos(b / 2)*std::cos(b / 2));
		}

		//规划身体姿态
		double s_even = aris::dynamic::even(param.totalCount, period_count + 1);
		double pq[7] = { 0,0,0,std::sin(s_even*b / 4)*up[0],std::sin(s_even*b / 4)*up[1] ,std::sin(s_even*b / 4)*up[2],std::cos(s_even*b / 4) };
		double pe[6];
		aris::dynamic::s_pq2pe(pq, pe);
		std::copy(pe + 3, pe + 6, Peb + 3);
	}
	//测试腰关节
	//double Wa = PI / 12 * std::sin(s);
	double Wa = 0;

	robot.SetPeb(Peb, beginMak);
	robot.SetWa(Wa);
	robot.SetPee(Pee, beginMak);

	if (param.count % 200 == 0)
	{
		double angle;
		robot.GetWa(angle);
		double deg = angle / PI * 180;
		//rt_printf("Set Waist angle: %f\n", Wa / PI * 180);
		//rt_printf("Get Waist angle: %f\n", deg);
	}

	return 2 * param.n * param.totalCount - param.count - 1;
}

