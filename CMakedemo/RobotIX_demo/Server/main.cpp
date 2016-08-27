﻿#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Type_III.h>
#include <Basic_Gait.h>

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


int main(int argc, char *argv[])
{
	std::string xml_address;

	if (argc <= 1)
	{
		std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
		xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
	}
	else if (std::string(argv[1]) == "III")
	{
		xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
	}
	else if (std::string(argv[1]) == "VIII")
	{
		xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
	}
    else if (std::string(argv[1]) == "IX")
    {
        xml_address = "/home/hex/Desktop/Robots_LJM/src/Robot_Type_III/resource/Robot_IX/Robot_IX.xml";
    }
    else
	{
		throw std::runtime_error("invalid robot name, please type in III or VIII");
	}
	
	auto &rs = aris::server::ControlServer::instance();
	

    rs.createModel<Robots::RobotTypeIII>();
	rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::Gait::basicParse, nullptr);
    rs.addCmd("ds", Robots::Gait::basicParse, nullptr);
    rs.addCmd("hm", Robots::Gait::basicParse, nullptr);
    rs.addCmd("rc", Robots::Gait::recoverParse, Robots::Gait::recoverGait);
    rs.addCmd("wk", Robots::Gait::walkParse, Robots::Gait::walkGait);
    rs.addCmd("ro", Robots::Gait::resetOriginParse, Robots::Gait::resetOriginGait);
    rs.addCmd("ec", Robots::Gait::extendChainParse, Robots::Gait::extendChainGait);
    //waist cmd
    rs.addCmd("rcw", Robots::Gait::recoverWaistParse, Robots::Gait::recoverWaistGait);
    rs.addCmd("aw", Robots::Gait::adjustWaistParse, Robots::Gait::adjustWaistGait);

	rs.open();

	rs.setOnExit([&]() 
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.LoadFile(xml_address.c_str());
		auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
		if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
		rs.model().saveXml(*model_xml_ele);
		
		aris::core::stopMsgLoop();
	});
	aris::core::runMsgLoop();
	
	

	return 0;
}