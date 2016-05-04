

#include <iostream>
#include "Robot_Type_I.h"

using namespace std;
using namespace Robots;
using namespace aris::dynamic;


int main()
{

	RobotTypeI rbt;

#ifdef WIN32
	rbt.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
	rbt.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif


	/*Compute inverse position kinematics in Ground coordinates*/
	


	char aaa;
	cin >> aaa;
	
	
	return 0;
}

