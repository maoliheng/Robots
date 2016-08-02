# Robots
在潘阳Robots-master基础上修改，RobotBase没改，新建了Robot_Type_III类。
匹配的Aris版本为v1.2.2。
xml文件修改了很多模型参数，请在新的xml文件上添加步态参数。
步态规划参考test/test_simple_walk工程，跟原来相比，需要注意以下两点：
    1）model的类型转换改为
        auto &robot = static_cast<Robots::RobotTypeIII &>(model);
    2）SetPeb和SetPee之间加一行SetWa，设置腰关节转角。若不使用腰关节，则可写成 SetWa(0)
        robot.SetPeb(Peb, beginMak);
	      robot.SetWa(Wa);
	      robot.SetPee(Pee, beginMak);
