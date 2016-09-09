#include "move_single_leg.h"

auto moveSingleLegParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
{
	mslParam param;

	for (auto &i : params)
	{
		if (i.first == "leg")
		{
			param.legID = std::stoi(i.second);
			if (param.legID < 0 || param.legID > 5) throw std::runtime_error("invalide param in parseRecover func");
			std::fill_n(param.active_motor, 18, false);
			std::fill_n(param.active_motor + param.legID * 3, 3, true);
		}
		else if (i.first == "totalCount")
		{
			param.totalCount = std::stoi(i.second);
		}
		else if (i.first == "x")
		{
			param.displacement[0] = std::stod(i.second);
		}
		else if (i.first == "y")
		{
			param.displacement[1] = std::stod(i.second);
		}
		else if (i.first == "z")
		{
			param.displacement[2] = std::stod(i.second);
		}
		else if (i.first == "absolute")
		{
			param.isAbsolute = true;
		}
		else if (i.first == "relative")
		{
			param.isAbsolute = false;
		}
	}

	msg.copyStruct(param);
}

auto moveSingleLegGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int
{
	auto &robot = static_cast<Robots::RobotTypeIII &>(model);
	auto &param = static_cast<const mslParam &>(param_in);

	static double beginPee[3], targetPee[3];
	static double beginWa;
	int id = param.legID;
	if (param.count == 0)
	{
		robot.pLegs[id]->GetPee(beginPee, robot.body());
		robot.GetWa(beginWa);

		//绝对坐标
		if (param.isAbsolute)
		{
			std::copy_n(param.displacement, 3, targetPee);
		}
		else
		{
			for (int i = 0; i < 3; ++i)
			{
				targetPee[i] = beginPee[i] + param.displacement[i];
			}
		}
	}

	double Pee[3];
	const double s = -0.5 * std::cos(PI * (param.count + 1) / param.totalCount) + 0.5; //s从0到1
	for (int i = 0; i < 3; ++i)
	{
		Pee[i] = beginPee[i] * (1 - s) + targetPee[i] * s;
	}
	robot.SetWa(beginWa);
	robot.pLegs[id]->SetPee(Pee);
}
