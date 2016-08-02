#include <Eigen/Eigen>

#include <aris.h>
#include <Robot_Type_III.h>
#include "test_gait.h"


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
	rbt.loadXml("G:\\Hexapod\\Robots_LJM\\src\\Robot_Type_III\\resource\\Robot_IX\\Robot_IX.xml");
	//rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII\\Robot_VIII.xml");
#endif
#ifdef UNIX
	rbt.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif
	
	const double beginEE[]{
		-0.3, -0.85, -0.65,
		-0.45, -0.85, 0,
		-0.3, -0.85, 0.65,
		0.3, -0.85, -0.65,
		0.45, -0.85, 0,
		0.3, -0.85, 0.65 };

	double beginPE[6]{ 0 };
	double beginWa{ 0 };
	
	/*
	Robots::WalkParam wk_param;
	wk_param.totalCount = 2000;
	wk_param.n = 1;
	wk_param.beta = 0.3;
	wk_param.alpha = 0;
	wk_param.d = 0.9;
	*/

	walkParam wk_param;

	rbt.SetPeb(beginPE);
	rbt.SetWa(beginWa);
	rbt.SetPee(beginEE);
	
	//auto result = rbt.simToAdams("G:\\Hexapod\\Robots_LJM_build\\simAdams\\test.cmd", Robots::walkGait, wk_param, 50);
	auto result = rbt.simToAdams("G:\\Hexapod\\Robots_LJM_build\\simAdams\\test.cmd", walkGait, wk_param, 50);

	result.saveToTxt("G:\\Hexapod\\Robots_LJM_build\\simAdams\\test");

	//rbt.saveXml("G:\\Hexapod\\Robots_LJM_build\\simAdams\\test.xml");
	
	return 0;
}