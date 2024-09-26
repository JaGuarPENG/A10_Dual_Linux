#include "robot.hpp"
#include "plan.hpp"

using namespace std;




cosCurve s1(1.0, 2 * PI, 0);




namespace robot
{

	auto ModelSetPos::prepareNrt()->void
	{
        for (auto& m : motorOptions()) m = aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelSetPos::executeRT()->int
	{
		/////example1//////

		double input_angle[12] =
		{ 0, 0, 5 * PI / 6, -PI / 3, -PI / 2, 0,
		0, 0, -5 * PI / 6, PI / 3, PI / 2, 0 };

		double input_angle1[6] =
		{ 0, 0, 5 * PI / 6, -PI / 3, -PI / 2, 0 };

		double input_angle2[6] =
		{ 0, 0, -5 * PI / 6, PI / 3, PI / 2, 0 };

		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 ->white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 ->blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);


		double eepA1[6] = { 0 };
		double eepBa[12] = { 0 };

		model_a1.setInputPos(input_angle1);
		if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
		model_a1.getOutputPos(eepA1);

		std::cout << "Arm1" << std::endl;
		aris::dynamic::dsp(1, 6, eepA1);
		
		dualArm.setInputPos(input_angle);
		if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;
		dualArm.getOutputPos(eepBa);

		std::cout << "dual" << std::endl;
		aris::dynamic::dsp(1, 12, eepBa);

		double Arm1_angle[6]{ 0 };
		double Dual_angle[12]{ 0 };


		model_a1.setOutputPos(eepA1);
		if (model_a1.inverseKinematics())std::cout << "inverse failed" << std::endl;
		model_a1.getInputPos(Arm1_angle);

		std::cout << "inverse Arm1" << std::endl;
		aris::dynamic::dsp(1, 6, Arm1_angle);


		dualArm.setOutputPos(eepBa);
		if (dualArm.inverseKinematics())std::cout << "inverse failed" << std::endl;
		dualArm.getInputPos(Dual_angle);
		
		std::cout << "inverse dual" << std::endl;
		aris::dynamic::dsp(1, 12, Dual_angle);


		//position
		double a1_stcp[6]{};
		double a2_stcp[12]{};
		//velocity
		double a1_vtcp[6]{};
		double a2_vtcp[12]{};

		//get ee
		auto& eeA1 = model_a1.generalMotionPool().at(0);
		auto& eeA2 = model_a2.generalMotionPool().at(0);

		eeA1.getP(a1_stcp);
		std::cout << "Arm1 ee pos" << std::endl;
		aris::dynamic::dsp(1, 6, a1_stcp);

		std::cout << "Arm1 ee vel" << std::endl;
		eeA1.getV(a1_vtcp);
		aris::dynamic::dsp(1, 6, a1_vtcp);

		eeA1.updP();

		//both arm move
		auto baMove = [&](double* pos_) {

			dualArm.setOutputPos(pos_);

			if (dualArm.inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[12]{ 0 };

			dualArm.getInputPos(x_joint);


			for (std::size_t i = 0; i < 12; ++i)
			{
				controller()->motorPool()[i].setTargetPos(x_joint[i]);
			}

		};

		//single arm move 1-->white 2-->blue
		auto saMove = [&](double* pos_, aris::dynamic::Model& model_, int type_) {

			model_.setOutputPos(pos_);

			if (model_.inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[6]{ 0 };

			model_.getInputPos(x_joint);

			if (type_ == 0)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i].setTargetPos(x_joint[i]);
				}
			}
			else if (type_ == 1)
			{
				for (std::size_t i = 0; i < 6; ++i)
				{
					controller()->motorPool()[i + 6].setTargetPos(x_joint[i]);
				}
			}
			else
			{
				throw std::runtime_error("Arm Type Error");
			}
		};



		double posTest[6]{ 0.402373, 0.124673, 0.136492, 6.283185, 0.000000, 3.141593 };
		saMove(posTest, model_a1, 1);

		double test[12]{};
		dualArm.getOutputPos(test);

		std::cout << "test" << std::endl;
		aris::dynamic::dsp(1, 12, test);





		////example1 end/////
	


		///gravity compensation test///
		GravComp gc;

		double force_data1[6]{ 0.126953, 0, 0.410156, 0.00234375, 0.00390625, 0.00117188 };
		double force_data2[6]{ -0.136719, 0.078125, 0.664062, 0.00507813, 0.00351563, 0.00117188 };
		double force_data3[6]{ -0.0976562, - 0.195312, 0.410156, 0.00507813, 0.00273437, 0.00117188};


		double pose1[9]{ -1.95049e-15, - 3.56239e-15, 1,
						-3.49721e-15, 1, 3.42057e-15,
						-1, - 3.48575e-15, - 4.9181e-15 };

		double pose2[9]{ -1, 3.22243e-15, - 3.33438e-15,
						3.23975e-15, 1, 3.23109e-15,
						3.45997e-15, 3.23109e-15, - 1 };

		double pose3[9]{ -1, 1.02138e-14, - 3.36091e-15,
						-3.37713e-15, - 3.17117e-15, 1,
						1.02143e-14, 1, 3.17117e-15 };

		double current_pose_a1[16] = { -1, 6.63139e-15, - 4.70536e-15, 0.32,
										1.33547e-15, 0.707107, 0.707107, 0.0516188,
										8.30063e-15, 0.707107, - 0.707107, 0.590381,
										0, 0, 0, 1 };
		double test_force1[6]{ -0.0976562, -0.0976562, 0.595703, 0.00546875, 0.00234375, 0.00117188 };

		double t_vector[9]{0};
		double f_vector[9]{0};

		double f_matrix[54]{0};
		double r_matrix[54]{0};

		double p_vector[6]{0};
		double l_vector[6]{0};

		double ee_rm_1[9]{0};
		double ee_rm_2[9]{0};
		double ee_rm_3[9]{0};

		double current_force[6]{0};
		double comp_f[6]{0};

		// aris::dynamic::s_pm2rm(ee_pm_1,ee_rm_1);
		// aris::dynamic::s_pm2rm(ee_pm_2,ee_rm_2);
		// aris::dynamic::s_pm2rm(ee_pm_3,ee_rm_3);

		gc.getTorqueVector(force_data1,force_data2,force_data3,t_vector);
		gc.getForceVector(force_data1,force_data2,force_data3,f_vector);

		gc.getFMatrix(force_data1,force_data2,force_data3,f_matrix);
		gc.getRMatrix(pose1,pose2,pose3,r_matrix);

		gc.getPLMatrix(f_matrix,t_vector,p_vector);
		gc.getPLMatrix(r_matrix,f_vector,l_vector);

		gc.getCompFT(current_pose_a1,l_vector,p_vector,comp_f);
		//getForceData(current_force);

		for(int i = 0; i<6; i++)
		{
			current_force[i] = test_force1[i]+comp_f[i];
		}

		aris::dynamic::dsp(1,6,current_force);


		return 0;
	}
	ModelSetPos::ModelSetPos(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_set\"/>");
	}
	ModelSetPos::~ModelSetPos() = default;



	auto ModelForward::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelForward::executeRT()->int
	{
		TCurve2 s1(1.0, 3.0, 20.0);
		s1.getCurveParam();
        static double init_pos[12]{};
        double input_angle[12]{};
        double ee_pos[12]{};
		
		if (count() == 1)
		{

            double begin_angle[12]{ 0 };

			for (int i = 0; i < 12; i++)
			{
				begin_angle[i] = controller()->motorPool()[i].targetPos();
			}

			aris::dynamic::dsp(1, 12, begin_angle);

			//this->master()->logFileRawName("move");
			mout() << "read init angle" << std::endl;

			modelBase()->setInputPos(begin_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			mout() << "input" << std::endl;

			modelBase()->getOutputPos(init_pos);

			aris::dynamic::dsp(1, 12, init_pos);

		}



			ee_pos[0] = init_pos[0] + 0.01 * s1.getTCurve(count());
		    ee_pos[1] = init_pos[1] - 0.01 * s1.getTCurve(count());

		    ee_pos[2] = init_pos[2];
		    ee_pos[3] = init_pos[3];
		    ee_pos[4] = init_pos[4];
		    ee_pos[5] = init_pos[5];

			ee_pos[6] = init_pos[6];
			ee_pos[7] = init_pos[7];
			ee_pos[8] = init_pos[8];
			ee_pos[9] = init_pos[9];
			ee_pos[10] = init_pos[10];
			ee_pos[11] = init_pos[11];


		modelBase()->setOutputPos(ee_pos);
		if (modelBase()->inverseKinematics())
		{
			throw std::runtime_error("Inverse Kinematics Position Failed!");
		}



		modelBase()->getInputPos(input_angle);

		for (int i = 0; i < 12; i++)
		{
			controller()->motorPool()[i].setTargetPos(input_angle[i]);
		}

		if (count() % 100 == 0)
		{
		mout() <<"Arm1:" << input_angle[0]*180/PI << "\t" << input_angle[1] * 180 / PI << "\t" << input_angle[2] * 180 / PI << "\t"
			<< input_angle[3] * 180 / PI << "\t" << input_angle[4] * 180 / PI << "\t" << input_angle[5] * 180 / PI <<"\n"
			<<"Arm2:" << input_angle[6] * 180 / PI << "\t" << input_angle[7] * 180 / PI << "\t" << input_angle[8] * 180 / PI
			<< input_angle[9] * 180 / PI << "\t" << input_angle[10] * 180 / PI << "\t" << input_angle[11] * 180 / PI << "\t" << count() << std::endl;
		}
		


		return 10000 - count();

	}
	ModelForward::ModelForward(const std::string& name)
	{

        aris::core::fromXmlString(command(),
            "<Command name=\"m_forward\">"
            "</Command>");
	}
	ModelForward::~ModelForward() = default;



	auto ModelTest::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelTest::executeRT() -> int
	{
		static double initPos[12]{ 0, 0, 5 * PI / 6, -5 * PI / 6, -90, 0,
							0, 0, -5 * PI / 6, 5 * PI / 6, 90, 0 };

		modelBase()->setInputPos(initPos);
		if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;

		double eePos[12]{ 0 };

		modelBase()->getOutputPos(eePos);
		std::cout << "init" << std::endl;
		aris::dynamic::dsp(1, 12, eePos);

		double finaleePos[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			finaleePos[i] = eePos[i];
		}

		finaleePos[0] = eePos[0] + 0.5;
		std::cout << "final" << std::endl;
		aris::dynamic::dsp(1, 12, finaleePos);



		modelBase()->setOutputPos(finaleePos);
		if (modelBase()->inverseKinematics())std::cout << "inverse failed" << std::endl;
		double finalpos[12] = { 0 };
		modelBase()->getInputPos(finalpos);

		aris::dynamic::dsp(1, 12, finalpos);

		double finalangle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			finalangle[i] = finalpos[i] * 180 / PI;
		}

		std::cout << "final angle" << std::endl;

		aris::dynamic::dsp(1, 12, finalangle);

		
		

		return 0;
	}
	ModelTest::ModelTest(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_test\"/>");
	}
	ModelTest::~ModelTest() = default;




	auto ModelGet::prepareNrt() -> void
	{
		option() |= NOT_PRINT_CMD_INFO;
	}
	auto ModelGet::executeRT() -> int
	{
        std::cout<<"size of slave pool:"<<ecMaster()->slavePool().size()<<std::endl;
        float force[6]={0};
        auto get_force_data = [&](float* data)
        {
            for(std::size_t i = 0; i<6; ++i)
            {
                ecMaster()->slavePool()[12].readPdo(0x6020, 0x01 + i, data + i, 32);
            }
        };

        get_force_data(force);
        std::cout<<"force data:"<<std::endl;
        aris::dynamic::dsp(1,6,force);

		return 0;
	}
	ModelGet::ModelGet(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_get\"/>");
	}
    ModelGet::~ModelGet() = default;




	auto ModelInit::prepareNrt()->void {

		for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


	}
	auto ModelInit::executeRT()->int {


		double eePos[12] = { 0 };

		static double move = 0.0001;
		static double tolerance = 0.00009;

		static double init_pos[12] = 
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, - PI / 2, 0, 
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2 };

		modelBase()->setInputPos(init_pos);

        if (modelBase()->forwardDynamics())
		{
			throw std::runtime_error("Forward Kinematics Position Failed!");
		}


		double current_angle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].targetPos();
		}


		auto motorsPositionCheck = [=]()
		{
			for(int i = 0; i < 12; i++)
			{
				if(std::fabs(current_angle[i]-init_pos[i])>=move)
				{
					return false;
				}
			}

			for (int i = 0; i < 12; i++)
			{
				controller()->motorPool()[i].setTargetPos(init_pos[i]);
			}

			return true;
		};



		for (int i = 0; i < 12; i++)
		{
			if (current_angle[i] <= init_pos[i] - move)
			{
				controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
			}
			else if (current_angle[i] >= init_pos[i] + move)
			{
				controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
			}
		}
	


	

		if (count() % 1000 == 0)
		{
			mout()<<current_angle[0]<<current_angle[1]<<current_angle[2]<<current_angle[3]<<current_angle[4]
			<<current_angle[5]<<current_angle[6]<<current_angle[7]<<current_angle[8]<<current_angle[9]
			<<current_angle[10]<<current_angle[11]<<std::endl;
		}


		if (motorsPositionCheck())
		{

			mout()<<"Back to Init Position"<<std::endl;
			modelBase()->setInputPos(current_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			
			mout()<<"current angle: \n"<<current_angle[0]<<current_angle[1]<<current_angle[2]<<current_angle[3]<<current_angle[4]
			<<current_angle[5]<<current_angle[6]<<current_angle[7]<<current_angle[8]<<current_angle[9]
			<<current_angle[10]<<current_angle[11]<<std::endl;

			modelBase()->getOutputPos(eePos);
			mout() << "current end position: \n" <<eePos[0]<<eePos[1]<<eePos[2]<<eePos[3]<<eePos[4]<<eePos[5]
			<<eePos[6]<<eePos[7]<<eePos[8]<<eePos[9]<<eePos[10]<<eePos[11]<<std::endl;


			return 0;
		}
		else
		{
			if(count()==28000)
			{
				mout()<<"Over Time"<<std::endl;
			}
			
			return 28000 - count();
		}
	}
	ModelInit::ModelInit(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_init\"/>");
	}
	ModelInit::~ModelInit() = default;

	auto ModelMoveX::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelMoveX::executeRT()->int
	{
		m_ = int32Param("model");
		d_ = doubleParam("distance");
		o_ = doubleParam("orientation");


		TCurve2 s1(1.0, 3.0, 20.0);
		s1.getCurveParam();
		static double init_pos[12]{};
		double input_angle[12]{};
		double ee_pos[12]{};
		double current_pos[12]{};
		double current_angle[12]{};

		if (count() == 1)
		{
			double begin_angle[12]{0};

			// double begin_angle[12]
			// { 0, 0, 5 * PI / 6, -PI / 3, -PI / 2, 0,
			// 0, 0, -5 * PI / 6, PI / 3, PI / 2, 0 };

			for (int i = 0; i < 12; i++)
			{
				begin_angle[i] = controller()->motorPool()[i].targetPos();
			}

			std::cout << "init angle:" << std::endl;
			aris::dynamic::dsp(1, 12, begin_angle);

			//this->master()->logFileRawName("move");
			std::cout << "read init angle" << std::endl;

			modelBase()->setInputPos(begin_angle);
			if (modelBase()->forwardKinematics())std::cout << "forward failed" << std::endl;
			modelBase()->getOutputPos(init_pos);

			std::cout << "init position:" << std::endl;
			aris::dynamic::dsp(1, 12, init_pos);

		}


		auto eemove = [&](double* pos_) {
			modelBase()->setOutputPos(pos_);
			if (modelBase()->inverseKinematics())
			{
				throw std::runtime_error("Inverse Kinematics Position Failed!");
			}


			double x_joint[12]{ 0 };


			modelBase()->getInputPos(x_joint);


			for (std::size_t i = 0; i < 12; ++i)
			{
				controller()->motorPool()[i].setTargetPos(x_joint[i]);
			}

		};


		modelBase()->getOutputPos(ee_pos);



		if (m_ == 0)
		{
			ee_pos[0] += 0.00001;

		}
		else if (m_ == 1)
		{
			ee_pos[6] += 0.00001;
		}
		else
		{
			std::cout << "model out of range; 0 ---> arm1 (white); 1 ---> arm2 (blue)" << std::endl;
		}


		eemove(ee_pos);

		modelBase()->getInputPos(input_angle);


		if (count() % 100 == 0)
		{
			mout() << "arm1:" << input_angle[0] * 180 / PI << "\t" << input_angle[1] * 180 / PI << "\t" << input_angle[2] * 180 / PI << "\t"
				<< input_angle[3] * 180 / PI << "\t" << input_angle[4] * 180 / PI << "\t" << input_angle[5] * 180 / PI << "\n"
				<< "arm2:" << input_angle[6] * 180 / PI << "\t" << input_angle[7] * 180 / PI << "\t" << input_angle[8] * 180 / PI
				<< input_angle[9] * 180 / PI << "\t" << input_angle[10] * 180 / PI << "\t" << input_angle[11] * 180 / PI << "\t" << std::endl;



		}


		//aris::dynamic::dsp(1,12,ee_pos);
		return (d_ * 100) - count();

	}
	ModelMoveX::ModelMoveX(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_x\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	<Param name=\"distance\" default=\"10.0\" abbreviation=\"d\"/>"
			"	<Param name=\"orientation\" default=\"1.0\" abbreviation=\"o\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ModelMoveX::~ModelMoveX() = default;





	struct ModelComP::Imp {
		bool target1_reached = false;
		bool target2_reached = false;
		bool target3_reached = false;
		bool target4_reached = false;

		bool stop_flag = false;
		int stop_count = 0;
		int stop_time = 5500;
		int current_stop_time = 0;

		int accumulation_count = 0;

		//temp data to stroage 10 times of actual force
		double temp_force1[6] = {0};
		double temp_force2[6] = {0};
		double temp_force3[6] = {0};

		double force_data_1[6]={0};
		double force_data_2[6]={0};
		double force_data_3[6]={0};

		double ee_pm_1[16]{0};
		double ee_pm_2[16]{0};
		double ee_pm_3[16]{0};

		double comp_f[6]{0};

		int m_;
	};

	auto ModelComP::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		


	}
	auto ModelComP::executeRT()->int
	{


		imp_ -> m_ = int32Param("model");
		
		

		static double move = 0.0001;

		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 -> white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 -> blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		//End Effector
		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));


		// Only One Arm Move Each Command
		auto jointMove = [&](double target_mp_[6], int m_)
		{
        	double mp[12];
			for (std::size_t i =(0 + 6*m_); i< (6 + 6*m_);++i)
			{
				
				if(controller()->motorPool()[i].actualPos() - target_mp_[i-6*m_] < 8/180*PI){
					if(controller()->motorPool()[i].actualPos() >= target_mp_[i-6*m_] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if(controller()->motorPool()[i].targetPos() <=target_mp_[i-6*m_] -0.0001){

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else{
						mp[i] = target_mp_[i-6*m_];

					}
				}else{
					if(controller()->motorPool()[i].actualPos() >= target_mp_[i-6*m_] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if(controller()->motorPool()[i].targetPos() <=target_mp_[i-6*m_] - 0.0001){

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else{
						mp[i] = target_mp_[i-6*m_];

					}

				}
            	controller()->motorPool()[i].setTargetPos(mp[i]);
        	}
    	};



		auto getForceData = [&](double* data_)
		{

			int raw_force[6]{ 0 };
			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[18].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

		};


		auto motorsPositionCheck = [](double current_sa_angle_[6], double target_pos_[6])
		{
			for (int i = 0; i < 6; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= move)
				{
					return false;
				}
			}

			return true;
		};

		auto caculateAvgForce = [=](double force_data_[6], double temp_force_[6], int count_)
		{
			if(count() < imp_->current_stop_time + imp_->stop_time)
				{
					
					if(count()%500 == 0 && imp_->accumulation_count<10)
					{
						double temp2[6]{1,2,3,4,5,6};
						//getForceData(temp2);

						for(int i = 0; i<6; i++)
						{
							temp_force_[i] = temp_force_[i] + temp2[i];
						}

						
						imp_->accumulation_count = imp_->accumulation_count + 1;
						mout()<<imp_->accumulation_count<<std::endl;
					}
					
				}
				else if(count() == imp_->current_stop_time + imp_->stop_time)
				{
					mout()<<"stop! "<<"count(): "<<count()<<std::endl;
					imp_->accumulation_count = 0;
					for(int i = 0; i<6; i++)
					{
						force_data_[i] = temp_force_[i]/10.0;
					}
					mout()<<"Force Data "<<count_<<'\n'<<force_data_[0]<<'\t'<<force_data_[1]<<'\t'<<force_data_[2]<<'\t'
					<<force_data_[3]<<'\t'<<force_data_[4]<<'\t'<<force_data_[5]<<std::endl;
				}
				else 
				{
					mout()<<"Flag Change "<<count_<<std::endl;
					imp_->stop_flag = false;
				}
		};




		static double init_pos[12] = 
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, - PI / 2, 0, 
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2 };

		double current_angle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}


		if(imp_->stop_flag)
		{
			if(imp_->stop_count == 1)
			{

				caculateAvgForce(imp_->force_data_1,imp_->temp_force1,1);

			}
			else if(imp_->stop_count == 2)
			{

				caculateAvgForce(imp_->force_data_2,imp_->temp_force2,2);

			}
			else if(imp_->stop_count == 3)
			{

				caculateAvgForce(imp_->force_data_3,imp_->temp_force3,3);

			}
			else
			{
				mout()<<"Stop Count Wrong: "<<imp_->stop_count<<" stop flag: "<<imp_->stop_flag<<std::endl;
				return 0;
			}
			
			return 80000 - count();
		}
		else
		{
			// arm 1 white arm
			if (imp_->m_ == 0)
			{
				static double angle1_1[6]{0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0};
			
				static double angle1_2[6]{0, 0, 5 * PI / 6, -PI / 2, -PI / 2, 0};

				static double angle1_3[6]{0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0};



				double current_sa_angle[6]{ 0 };
				std::copy(current_angle, current_angle + 6, current_sa_angle);

				if (count() % 1000 == 0)
				{
					mout()<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
					<<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;

				}


				

				if(!imp_->target1_reached)
				{
					model_a1.setInputPos(angle1_1);
					if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
					
					jointMove(angle1_1,0);

					if(motorsPositionCheck(current_sa_angle, angle1_1))
					{
						mout() << "Target 1 Reached" << std::endl;
						
						
						eeA1.getMpm(imp_->ee_pm_1);

						imp_->target1_reached = true;
						imp_->stop_count = 1;
						imp_->current_stop_time = count();
						imp_ -> stop_flag = true;
						mout()<<"current stop time: "<<imp_->current_stop_time<<std::endl;
					}

				}
				else if(imp_->target1_reached && !imp_->target2_reached)
				{
					model_a1.setInputPos(angle1_2);
					if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
					
					jointMove(angle1_2,0);

					if(motorsPositionCheck(current_sa_angle, angle1_2))
					{
						mout() << "Target 2 Reached" << std::endl;
						
						eeA1.getMpm(imp_->ee_pm_2);

						imp_->target2_reached = true;
						imp_->stop_count = 2;
						imp_->current_stop_time = count();
						imp_ -> stop_flag = true;
						mout()<<"current stop time: "<<imp_->current_stop_time<<std::endl;
					}
				}
				else if(imp_->target2_reached && !imp_->target3_reached)
				{
					model_a1.setInputPos(angle1_3);
					if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
					
					jointMove(angle1_3,0);

					if(motorsPositionCheck(current_sa_angle, angle1_3))
					{
						mout() << "Target 3 Reached" << std::endl;
						
						eeA1.getMpm(imp_->ee_pm_3);

						imp_->target3_reached = true;
						imp_->stop_count = 3;
						imp_->current_stop_time = count();
						imp_ -> stop_flag = true;
						mout()<<"current stop time: "<<imp_->current_stop_time<<std::endl;

					}
				}
				else if(imp_->target3_reached && !imp_->target4_reached)
				{
					// Back To Init
					model_a1.setInputPos(angle1_1);
					if (model_a1.forwardKinematics())std::cout << "forward failed" << std::endl;
					
					jointMove(angle1_1,0);

					if(motorsPositionCheck(current_sa_angle, angle1_1))
					{
						mout() << "Back To Init Pos" << std::endl;
						mout()<<"Current Angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
						<<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
						
						imp_->target4_reached = true;

					}

				}
				else if(imp_->target1_reached && imp_->target2_reached && imp_->target3_reached && imp_->target4_reached)
				{
					double t_vector[9]{0};
					double f_vector[9]{0};

					double f_matrix[54]{0};
					double r_matrix[54]{0};

					double p_vector[6]{0};
					double l_vector[6]{0};

					double ee_rm_1[9]{0};
					double ee_rm_2[9]{0};
					double ee_rm_3[9]{0};

					double current_force[6]{0};

					aris::dynamic::s_pm2rm(imp_->ee_pm_1,ee_rm_1);
					aris::dynamic::s_pm2rm(imp_->ee_pm_2,ee_rm_2);
					aris::dynamic::s_pm2rm(imp_->ee_pm_3,ee_rm_3);

					GravComp gc;

					gc.getTorqueVector(imp_->force_data_1,imp_->force_data_2,imp_->force_data_3,t_vector);
					gc.getForceVector(imp_->force_data_1,imp_->force_data_2,imp_->force_data_3,f_vector);

					gc.getFMatrix(imp_->force_data_1,imp_->force_data_2,imp_->force_data_3,f_matrix);
					gc.getRMatrix(ee_rm_1,ee_rm_3,ee_rm_3,r_matrix);

					gc.getPLMatrix(f_matrix,t_vector,p_vector);
					gc.getPLMatrix(r_matrix,f_vector,l_vector);

					double current_ee_pm[16]{0};
					eeA1.getMpm(current_ee_pm);

					gc.getCompFT(current_ee_pm,l_vector,p_vector,imp_->comp_f);
					getForceData(current_force);

					mout()<<"Current Force After Compensation:"<<'\n'<<current_force[0]+imp_->comp_f[0]<<'\t'<<current_force[1]+imp_->comp_f[1]<<'\t'
					<<current_force[2]+imp_->comp_f[2]<<'\t'<<current_force[3]+imp_->comp_f[3]<<'\t'
					<<current_force[4]+imp_->comp_f[4]<<'\t'<<current_force[5]+imp_->comp_f[5]<<std::endl;

					mout()<<"Current End Pos:"<<'\n'<<current_ee_pm[0]<<'\t'<<current_ee_pm[1]<<'\t'
					<<current_ee_pm[2]<<'\t'<<current_ee_pm[3]<<'\t'
					<<current_ee_pm[4]<<'\t'<<current_ee_pm[5]<<std::endl;

					return 0;
				}
				
				if (count() == 80000)
				{

					mout() << "Over Time" << std::endl;
				}

				return 80000 - count();
				


			}

			// arm 2 blue arm
			else if(imp_->m_ == 1)
			{
				// 	Init Pos
				static double angle2_1[12]{0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2};
			
				static double angle2_2[12]{0, 0, -5 * PI / 6, PI / 2, PI / 2, PI / 2};

				static double angle2_3[12]{0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, PI / 2};

				double current_sa_angle[6]{ 0 };

				std::copy(current_angle + 6, current_angle + 12, current_sa_angle);

				if (count() % 1000 == 0)
				{
					mout()<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
					<<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;

				}


				

				if(!imp_->target1_reached)
				{
					model_a2.setInputPos(angle2_1);
					if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;
					
					jointMove(angle2_1,1);

					if(motorsPositionCheck(current_sa_angle, angle2_1))
					{
						mout() << "Target 1 Reached" << std::endl;
						
						
						eeA2.getMpm(imp_->ee_pm_1);

						imp_->target1_reached = true;
						imp_->stop_count = 1;
						imp_->current_stop_time = count();
						imp_ -> stop_flag = true;
						mout()<<"current stop time: "<<imp_->current_stop_time<<std::endl;
					}

				}
				else if(imp_->target1_reached && !imp_->target2_reached)
				{
					model_a2.setInputPos(angle2_2);
					if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;
					
					jointMove(angle2_2,1);

					if(motorsPositionCheck(current_sa_angle, angle2_2))
					{
						mout() << "Target 2 Reached" << std::endl;
						
						eeA2.getMpm(imp_->ee_pm_2);

						imp_->target2_reached = true;
						imp_->stop_count = 2;
						imp_->current_stop_time = count();
						imp_ -> stop_flag = true;
						mout()<<"current stop time: "<<imp_->current_stop_time<<std::endl;
					}
				}
				else if(imp_->target2_reached && !imp_->target3_reached)
				{
					model_a2.setInputPos(angle2_3);
					if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;
					
					jointMove(angle2_3,1);

					if(motorsPositionCheck(current_sa_angle, angle2_3))
					{
						mout() << "Target 3 Reached" << std::endl;
						
						eeA2.getMpm(imp_->ee_pm_3);

						imp_->target3_reached = true;
						imp_->stop_count = 3;
						imp_->current_stop_time = count();
						imp_ -> stop_flag = true;
						mout()<<"current stop time: "<<imp_->current_stop_time<<std::endl;

					}
				}
				else if(imp_->target3_reached && !imp_->target4_reached)
				{
					// Back To Init
					model_a2.setInputPos(angle2_1);
					if (model_a2.forwardKinematics())std::cout << "forward failed" << std::endl;
					
					jointMove(angle2_1,1);

					if(motorsPositionCheck(current_sa_angle, angle2_1))
					{
						mout() << "Back To Init Pos" << std::endl;
						mout()<<"Current Angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
						<<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
						
						imp_->target4_reached = true;

					}

				}
				else if(imp_->target1_reached && imp_->target2_reached && imp_->target3_reached && imp_->target4_reached)
				{
					double t_vector[9]{0};
					double f_vector[9]{0};

					double f_matrix[54]{0};
					double r_matrix[54]{0};

					double p_vector[6]{0};
					double l_vector[6]{0};

					double ee_rm_1[9]{0};
					double ee_rm_2[9]{0};
					double ee_rm_3[9]{0};

					double current_force[6]{0};

					aris::dynamic::s_pm2rm(imp_->ee_pm_1,ee_rm_1);
					aris::dynamic::s_pm2rm(imp_->ee_pm_2,ee_rm_2);
					aris::dynamic::s_pm2rm(imp_->ee_pm_3,ee_rm_3);

					GravComp gc;

					gc.getTorqueVector(imp_->force_data_1,imp_->force_data_2,imp_->force_data_3,t_vector);
					gc.getForceVector(imp_->force_data_1,imp_->force_data_2,imp_->force_data_3,f_vector);

					gc.getFMatrix(imp_->force_data_1,imp_->force_data_2,imp_->force_data_3,f_matrix);
					gc.getRMatrix(ee_rm_1,ee_rm_3,ee_rm_3,r_matrix);

					gc.getPLMatrix(f_matrix,t_vector,p_vector);
					gc.getPLMatrix(r_matrix,f_vector,l_vector);

					double current_ee_pm[16]{0};
					eeA2.getMpm(current_ee_pm);

					gc.getCompFT(current_ee_pm,l_vector,p_vector,imp_->comp_f);
					//getForceData(current_force);

					mout()<<"Current Force After Compensation:"<<'\n'<<current_force[0]+imp_->comp_f[0]<<'\t'<<current_force[1]+imp_->comp_f[1]<<'\t'
					<<current_force[2]+imp_->comp_f[2]<<'\t'<<current_force[3]+imp_->comp_f[3]<<'\t'
					<<current_force[4]+imp_->comp_f[4]<<'\t'<<current_force[5]+imp_->comp_f[5]<<std::endl;

					mout()<<"Current End Pos:"<<'\n'<<current_ee_pm[0]<<'\t'<<current_ee_pm[1]<<'\t'
					<<current_ee_pm[2]<<'\t'<<current_ee_pm[3]<<'\t'
					<<current_ee_pm[4]<<'\t'<<current_ee_pm[5]<<std::endl;

					return 0;
				}
				
				if (count() == 80000)
				{

					mout() << "Over Time" << std::endl;
				}

				return 80000 - count();
				




			}
			// wrong model
			else
			{
				mout()<<"Wrong Model"<<std::endl;
				throw std::runtime_error("Arm Type Error");
				return 0;
			}
		}
		









		

	}
	ModelComP::ModelComP(const std::string& name) : imp_(new Imp)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_comp\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ModelComP::~ModelComP() = default;




	ARIS_REGISTRATION{
		aris::core::class_<ModelInit>("ModelInit")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelForward>("ModelForward")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelGet>("ModelGet")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelMoveX>("ModelMoveX")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelSetPos>("ModelSetPos")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ModelComP>("ModelComP")
			.inherit<aris::plan::Plan>();

	}





}





