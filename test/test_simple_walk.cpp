#include <Eigen/Eigen>

#include <aris.h>
#include <Robot_Type_I.h>

#include <type_traits>



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
	Robots::RobotTypeI rbt;


#ifdef WIN32
	rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
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

	rbt.SetPeb(beginPE);
	rbt.SetPee(beginEE);

	/*
	rbt.SetFixFeet("000000");

	std::vector<std::array<double,18> > pin_vec(9901), vin_vec(9901), ain_vec(9901), fin_vec(9901), fee_vec(9901);

	aris::dynamic::dlmread("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_calibration\\pos_for_test.txt", pin_vec.data()->data());
	aris::dynamic::dlmread("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_calibration\\vel_for_test.txt", vin_vec.data()->data());
	aris::dynamic::dlmread("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_calibration\\acc_for_test.txt", ain_vec.data()->data());
	aris::dynamic::dlmread("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_calibration\\fce_for_test.txt", fin_vec.data()->data());

	rbt.SetVb(beginPE);
	rbt.SetAb(beginPE);

	for (auto i = 0; i < 9901; ++i)
	{
		std::cout << i << std::endl;
		
		rbt.SetPin(pin_vec.at(i).data());
		rbt.SetVin(vin_vec.at(i).data());
		rbt.SetAin(ain_vec.at(i).data());

		rbt.FastDyn();

		double f[18];
		rbt.GetFin(f);
		
		double f_sta[18];
		std::copy(fin_vec.at(i).begin(), fin_vec.at(i).end(), f_sta);
		aris::dynamic::s_va(18, -1, f, 1, f_sta, 1);

		rbt.SetFinSta(f_sta);

		rbt.GetFeeSta(fee_vec.at(i).data());
	}

	aris::dynamic::dlmwrite("D:\\kuaipan\\Hexapod_Robot\\Robot_III\\m_calibration\\fee_after_test.txt",fee_vec);
	*/





	Robots::WalkParam wk_param;
	wk_param.totalCount = 2000;
	wk_param.n = 1;
	wk_param.beta = 0.3;
	wk_param.alpha = 0.3;
	wk_param.d = 0.9;
	
	//auto result = rbt.simKin(Robots::walkGait, wk_param, 1);

	auto result = rbt.simToAdams("C:\\Users\\yang\\Desktop\\test.cmd", Robots::walkGait, wk_param, 50);
	result.saveToTxt("C:\\Users\\yang\\Desktop\\test");
	rbt.saveXml("C:\\Users\\yang\\Desktop\\test.xml");
	
	/*
	Robots::WalkParam wk_param;
	wk_param.totalCount = 2000;
	wk_param.n = 3;
	wk_param.beta = 0.3;
	wk_param.d = 0.5;
	aris::dynamic::SimResult result;
	rbt.SetPeb(beginPE);
	rbt.SetPee(beginEE);
	rbt.simKin(Robots::walk, wk_param, result, true);
	result.saveToTxt("C:\\Users\\yang\\Desktop\\test");


	auto d = 0.5;

	double s[1000];

	for (int i = 0; i < 1000;++i)
	s[i]=aris::dynamic::s_interp(1000, i + 1, 0, d/2, 0, d/1000);

	aris::dynamic::dlmwrite("C:\\Users\\yang\\Desktop\\s.txt",s, 1, 1000);

	/*
	SimpleWalkParam param;
	rbt.GetPee(param.beginPee);
	rbt.GetPeb(param.beginPeb);

	rbt.SimByMatlab("C:\\Users\\yang\\Desktop\\test\\", SimpleWalk, param);

	rbt.SimScriptClear();
	rbt.SimScriptSetTopologyA();
	rbt.SimScriptSimulate(500, 10);
	rbt.SimScriptSetTopologyB();
	rbt.SimScriptSimulate(500, 10);
	rbt.SimByAdams("C:\\Users\\yang\\Desktop\\test.cmd", SimpleWalk, param, 10, true);
	rbt.SimByAdamsResultAt(101);

	double fin[18];
	rbt.GetFinDyn(fin);
	aris::dynamic::dsp(fin, 18, 1);

	{
	rbt.ClbPre();
	rbt.ClbUpd();
	rbt.ClbSetInverseMethod([](int n, double *A)
	{
	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >Am(A, n, n);
	auto im = Am.inverse();
	Am = im;
	});
	rbt.ForEachMotion([](aris::dynamic::Motion *p)
	{
	p->SetMotFce(p->MotFce());
	});

	aris::dynamic::Matrix D(rbt.ClbDimM(), rbt.ClbDimN());
	aris::dynamic::Matrix b(rbt.ClbDimM(), 1);
	aris::dynamic::Matrix x(rbt.ClbDimN(), 1);

	rbt.ClbMtx(D.Data(), b.Data());
	rbt.ClbUkn(x.Data());

	auto result = D*x;
	result.dsp();
	b.dsp();
	}

	*/
	return 0;
}