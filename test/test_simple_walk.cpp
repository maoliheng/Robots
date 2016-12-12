#include <Eigen/Eigen>

#include <aris.h>
#include <Robot_Type_III.h>
#include <Basic_Gait.h>
#include "walk_on_ramp.h"


Robots::RobotTypeIII rbt;

int main_test(int argc, char *argv[]);
int main(int argc, char *argv[])
{
	try
	{
		main_test(argc, argv);
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}


	std::cout << "finished" << std::endl;

	char aaa;
	std::cin >> aaa;
	return 0;
}

int main_test(int argc, char *argv[])
{
	
#ifdef WIN32
	rbt.loadXml("C:\\Robots2\\resource\\Robot_Type_III\\Robot_IX\\Robot_IX.xml");
	//rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII\\Robot_VIII.xml");
#endif
#ifdef UNIX
	rbt.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif
	
	//double beginEE[]{
	//	-0.3, -0.477, -0.532,
	//	-0.45, -0.85, 0,
	//	-0.3, -1.223, 0.532,
	//	0.3, -0.477, -0.532,
	//	0.45, -0.85, 0,
	//	0.3, -1.223, 0.532 };
	double beginEE[]{
		-0.3, -0.547, -0.532,
		-0.45, -0.92, 0,
		-0.3, -1.293, 0.532,
		0.3, -0.547, -0.532,
		0.45, -0.92, 0,
		0.3, -1.293, 0.532 };

	double beginPE[6]{ 0 };
	double beginWa{ PI/180*45 };

	//ÅÀÆÂ²âÊÔ
	//double phi = PI / 9; //²àÆÂ½Ç¶È
	//for (int i = 0; i < 6; ++i)
	//{
	//	beginEE[3 * i + 1] += beginEE[3 * i] * std::tan(phi);
	//}
	//beginPE[3] = phi;
	
	worParam wor_param;
	wor_param.totalCount = 2000;
	wor_param.n = 4;
	wor_param.angle = 35;
	wor_param.d = 0.3;
	wor_param.h = 0.05;

	//walkParam wk_param;
	//wk_param.totalCount = 2000;
	//wk_param.n = 2;
	//wk_param.beta = 0;
	//wk_param.alpha = 0;
	//wk_param.d = 0.5;

	//Robots::Gait::AdjustWaistParam aw_param;
	//aw_param.angle = 0;

	//creepingParam cp_param;
	//cp_param.n = 3;
	//cp_param.d = 0.4;

	rbt.SetPeb(beginPE);
	rbt.SetWa(beginWa);
	rbt.SetPee(beginEE);
	
	auto result = rbt.simToAdams("D:\\Lab\\Models\\Adams\\RobotIX\\walking_on_ramp.cmd", walkOnRampGait, wor_param, 50);
	//auto result = rbt.simToAdams("G:\\Hexapod\\Robots_LJM_build\\simAdams\\creeping.cmd", creepingGait, cp_param, 50);

	result.saveToTxt("D:\\Lab\\Models\\Adams\\RobotIX\\walking_on_ramp");

	//rbt.saveXml("G:\\Hexapod\\Robots_LJM_build\\simAdams\\test.xml");
	
	return 0;
}