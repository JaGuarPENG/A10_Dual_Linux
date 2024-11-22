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

		static double init_angle[12] =
			{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

		static double angle1[12] =
			{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 ,
			0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

		static double angle2[12] =
			{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 3, 0 ,
			0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

		static double angle3[12] =
			{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 ,
			0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };

		
		if(count()<=2000)
		{
			dualArm.setInputPos(init_angle);
			if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

			for(int i = 0; i<12; i++)
			{
				controller()->motorPool()[i].setTargetPos(init_angle[i]);
			}
		}
		else if(count() > 2000 && count() <= 4000)
		{
			dualArm.setInputPos(angle1);
			if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

			for(int i = 0; i<12; i++)
			{
				controller()->motorPool()[i].setTargetPos(angle1[i]);
			}
		}
		else if(count() > 4000 && count() <=6000 )
		{
			dualArm.setInputPos(angle2);
			if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

			for(int i = 0; i<12; i++)
			{
				controller()->motorPool()[i].setTargetPos(angle2[i]);
			}
		}
		else if(count() > 6000 && count() <=8000 )
		{
			dualArm.setInputPos(angle3);
			if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

			for(int i = 0; i<12; i++)
			{
				controller()->motorPool()[i].setTargetPos(angle3[i]);
			}
		}


		return 8000 - count();
	}
	ModelSetPos::ModelSetPos(const std::string& name)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_set\"/>");
	}
	ModelSetPos::~ModelSetPos() = default;


	struct ModelForward::Imp {

			bool init = false;

		};
	auto ModelForward::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
	}
	auto ModelForward::executeRT()->int
	{
		//dual transform modelbase into multimodel
		auto& dualArm = dynamic_cast<aris::dynamic::MultiModel&>(modelBase()[0]);
		//at(0) -> Arm1 ->white
		auto& arm1 = dualArm.subModels().at(0);
		//at(1) -> Arm2 ->blue
		auto& arm2 = dualArm.subModels().at(1);

		//transform to model
		auto& model_a1 = dynamic_cast<aris::dynamic::Model&>(arm1);
		auto& model_a2 = dynamic_cast<aris::dynamic::Model&>(arm2);

		auto& eeA1 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a1.generalMotionPool().at(0));
		auto& eeA2 = dynamic_cast<aris::dynamic::GeneralMotion&>(model_a2.generalMotionPool().at(0));

		static double tolerance = 0.0001;
		static double init_angle[12] = 
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, - PI / 2, 0, 
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

		double current_angle[12]{0};

		auto daJointMove = [&](double target_mp_[12])
		{
			double mp[12];

			for (std::size_t i = 0; i < 12; ++i)
			{

				if (controller()->motorPool()[i].actualPos() - target_mp_[i] < 8 / 180 * PI) {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}
				}
				else {
					if (controller()->motorPool()[i].actualPos() >= target_mp_[i] + 0.0001)
					{

						mp[i] = controller()->motorPool()[i].targetPos() - 0.0001;
					}
					else if (controller()->motorPool()[i].targetPos() <= target_mp_[i] - 0.0001) {

						mp[i] = controller()->motorPool()[i].targetPos() + 0.0001;
					}
					else {
						mp[i] = target_mp_[i];

					}

				}
				controller()->motorPool()[i].setTargetPos(mp[i]);
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
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



		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}
		
		if(!imp_->init)
		{
			dualArm.setInputPos(init_angle);
			if(dualArm.forwardKinematics())
			{
				std::cout<<"Forward Error"<<std::endl;
			}
			daJointMove(init_angle);
			if(motorsPositionCheck(current_angle,init_angle,12))
			{
				mout()<<"Init Complete"<<std::endl;
				imp_->init = true;
			}

		}
		else
		{	//Only Arm 1
			double current_pos[6]{0};
			eeA1.getP(current_pos);

			current_pos[0] +=0.00001;

			if (count() % 1000 == 0)
			{

				mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
					<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

			}

			saMove(current_pos, model_a1, 0);
		}


		return 40000 - count();

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

		static double tolerance = 0.0001;

		aris::Size total_count;

		static double joint_vel_max = 15;
		static double joint_acc_max = 100;
		static double joint_dec_max = 100;

		static double init_pos[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

		double current_pos_a1[6]{0};
		double current_pos_a2[6]{0};

		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};
	

		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
		};




		dualArm.setInputPos(init_pos);

		if (dualArm.forwardKinematics())
		{
			throw std::runtime_error("Forward Kinematics Position Failed!");
		}

		eeA1.getP(current_pos_a1);
		eeA2.getP(current_pos_a2);

		double current_angle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].targetPos();
			controller()->motorPool()[i].setTargetPos(init_pos[i]);
		}



		//daJointMove(init_pos);





		if (count() % 1000 == 0)
		{
			mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t' << current_angle[3] << '\t' << current_angle[4] << '\t'
				<< current_angle[5] << '\t' << current_angle[6] << '\t' << current_angle[7] << '\t' << current_angle[8] << '\t' << current_angle[9] << '\t'
				<< current_angle[10] << '\t' << current_angle[11] << '\t' << std::endl;
		}


		if (motorsPositionCheck(current_angle, init_pos, 12))
		{

			mout() << "Back to Init Position" << std::endl;
			dualArm.setInputPos(current_angle);
			if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

			mout() << "current angle: \n" << current_angle[0] <<'\t' << current_angle[1] << '\t' << current_angle[2] << '\t' << current_angle[3] << '\t' << current_angle[4] << '\t'
				<< current_angle[5] << '\t' << current_angle[6] << '\t' << current_angle[7] << '\t' << current_angle[8] << '\t' << current_angle[9] << '\t'
				<< current_angle[10] << '\t' << current_angle[11] << '\t' << std::endl;

			mout() << "current a1 pos: \n" << current_pos_a1[0] <<'\t' << current_pos_a1[1] << '\t' << current_pos_a1[2] << '\t' << current_pos_a1[3] << '\t' << current_pos_a1[4] << '\t'
				<< current_pos_a1[5] << std::endl;

			mout() << "current a2 pos: \n" << current_pos_a2[0] <<'\t' << current_pos_a2[1] << '\t' << current_pos_a2[2] << '\t' << current_pos_a2[3] << '\t' << current_pos_a2[4] << '\t'
				<< current_pos_a2[5] << std::endl;

			return 0;
		}
		else
		{
			if (count() == 80000)
			{
				mout() << "Over Time" << std::endl;
			}

			return 80000 - count();
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

			bool init = false;

			bool stop_flag = false;
			int stop_count = 0;
			int stop_time = 2200;
			int current_stop_time = 0;

			int accumulation_count = 0;

			//temp data to stroage 10 times of actual force
			

			// For Arm 1
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			double arm1_temp_force_1[6] = { 0 };
			double arm1_temp_force_2[6] = { 0 };
			double arm1_temp_force_3[6] = { 0 };

			double arm1_force_data_1[6] = { 0 };
			double arm1_force_data_2[6] = { 0 };
			double arm1_force_data_3[6] = { 0 };

			double arm1_ee_pm_1[16]{ 0 };
			double arm1_ee_pm_2[16]{ 0 };
			double arm1_ee_pm_3[16]{ 0 };

			double arm1_comp_f[6]{ 0 };
			double arm1_init_force[6]{ 0 };


			//For Arm 2
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			double arm2_temp_force_1[6] = { 0 };
			double arm2_temp_force_2[6] = { 0 };
			double arm2_temp_force_3[6] = { 0 };

			double arm2_force_data_1[6] = { 0 };
			double arm2_force_data_2[6] = { 0 };
			double arm2_force_data_3[6] = { 0 };

			double arm2_ee_pm_1[16]{ 0 };
			double arm2_ee_pm_2[16]{ 0 };
			double arm2_ee_pm_3[16]{ 0 };

			double arm2_comp_f[6]{ 0 };
			double arm2_init_force[6]{ 0 };

		};
	auto ModelComP::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;



	}
	auto ModelComP::executeRT()->int
	{


		GravComp gc;
		static double tolerance = 0.0001;

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
		auto saJointMove = [&](double target_mp_[6], int m_)
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0 + (6 * m_); i < 6 + (6 * m_); i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};

		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};

		//Ethercat Warning
		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (int i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
				{
					mout() << "Force Sensor Error" << std::endl;
				}


				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}
			if (!init_)
			{
				if (m_ == 0)
				{
					mout() << "Compensate Init Force A1" << std::endl;
					mout() << "Init 1 : " << imp_->arm1_init_force[0] << '\t' << imp_->arm1_init_force[1] << '\t' << imp_->arm1_init_force[2] << '\t'
						<< imp_->arm1_init_force[3] << '\t' << imp_->arm1_init_force[4] << '\t' << imp_->arm1_init_force[5] << std::endl;
				}
				else if (m_ == 1)
				{
					mout() << "Compensate Init Force A2" << std::endl;
					mout() << "Init 2 : " << imp_->arm2_init_force[0] << '\t' << imp_->arm2_init_force[1] << '\t' << imp_->arm2_init_force[2] << '\t'
						<< imp_->arm2_init_force[3] << '\t' << imp_->arm2_init_force[4] << '\t' << imp_->arm2_init_force[5] << std::endl;
				}
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
		};

		auto caculateAvgForce = [=](double arm1_force_data_[6], double arm2_force_data_[6], double arm1_temp_force_[6], double arm2_temp_force_[6], int count_)
		{
			if (count() < imp_->current_stop_time + imp_->stop_time)
			{

				if (count() % 200 == 0 && imp_->accumulation_count < 10)
				{
					double temp1[6]{ 1,2,3,4,5,6 };
					double temp2[6]{ 1,2,3,4,5,6 };

					//getForceData(temp1, 0, imp_->init);
					//getForceData(temp2, 1, imp_->init);

					for (int i = 0; i < 6; i++)
					{
						arm1_temp_force_[i] += temp1[i];
						arm2_temp_force_[i] += temp2[i];
					}


					imp_->accumulation_count = imp_->accumulation_count + 1;
					mout() << imp_->accumulation_count << std::endl;
				}

			}
			else if (count() == imp_->current_stop_time + imp_->stop_time)
			{
				mout() << "stop! " << "count(): " << count() << std::endl;
				imp_->accumulation_count = 0;
				for (int i = 0; i < 6; i++)
				{
					arm1_force_data_[i] = arm1_temp_force_[i] / 10.0;
					arm2_force_data_[i] = arm2_temp_force_[i] / 10.0;
				}
				mout() << "Arm 1 Force Data " << count_ << '\n' << arm1_force_data_[0] << '\t' << arm1_force_data_[1] << '\t' << arm1_force_data_[2] << '\t'
					<< arm1_force_data_[3] << '\t' << arm1_force_data_[4] << '\t' << arm1_force_data_[5] << std::endl;
				mout() << "Arm 2 Force Data " << count_ << '\n' << arm2_force_data_[0] << '\t' << arm2_force_data_[1] << '\t' << arm2_force_data_[2] << '\t'
					<< arm2_force_data_[3] << '\t' << arm2_force_data_[4] << '\t' << arm2_force_data_[5] << std::endl;
			}
			else
			{
				mout() << "Flag Change " << count_ << std::endl;
				imp_->stop_flag = false;
			}
		};

		
		double current_angle[12] = { 0 };

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (imp_->stop_flag)
		{
			if (imp_->stop_count == 1)
			{

				caculateAvgForce(imp_->arm1_force_data_1, imp_->arm2_force_data_1, imp_->arm1_temp_force_1, imp_->arm2_temp_force_1, 1);
				

			}
			else if (imp_->stop_count == 2)
			{

				caculateAvgForce(imp_->arm1_force_data_2, imp_->arm2_force_data_2, imp_->arm1_temp_force_2, imp_->arm2_temp_force_2, 2);

			}
			else if (imp_->stop_count == 3)
			{

				caculateAvgForce(imp_->arm1_force_data_3, imp_->arm2_force_data_3, imp_->arm1_temp_force_3, imp_->arm2_temp_force_3, 3);

			}
			else
			{
				mout() << "Stop Count Wrong: " << imp_->stop_count << " stop flag: " << imp_->stop_flag << std::endl;
				return 0;
			}

			return 80000 - count();
		}
		else
		{
			static double init_angle[12] =
			{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
			0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

			static double angle1[12] =
			{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 ,
			0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

			static double angle2[12] =
			{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 3, 0 ,
			0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

			static double angle3[12] =
			{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 ,
			0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };


			//// Arm 1 Angle
			//static double init_angle1[6]{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 };

			//static double angle1_1[6]{ 0, 0, 5 * PI / 6, -17 * PI / 18, -PI / 2, 0 };

			//static double angle1_2[6]{ 0, 0, 5 * PI / 6, -PI / 2, -PI / 2, 0 };

			//static double angle1_3[6]{ 0, 0, 5 * PI / 6, -2 * PI / 3, -2 * PI / 3, 0 };




			//// Arm 2 Angle
			//static double init_angle2[6]{ 0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };

			//static double angle2_1[6]{ 0, 0, -5 * PI / 6, 17 * PI / 18, PI / 2, 0 };

			//static double angle2_2[6]{ 0, 0, -5 * PI / 6, PI / 2, PI / 3, 0 };

			//static double angle2_3[6]{ 0, 0, -5 * PI / 6, 2 * PI / 3, 2 * PI / 3, 0 };

			if (!imp_->init)
			{
				dualArm.setInputPos(init_angle);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(init_angle);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(init_angle[i]);
				}

				if (motorsPositionCheck(current_angle, init_angle, 12))
				{

					//getForceData(imp_->arm1_init_force, 0, imp_->init);
					//getForceData(imp_->arm2_init_force, 1, imp_->init);

					mout() << "Init Complete" << std::endl;

					imp_->init = true;
				}

			}

			if (!imp_->target1_reached && imp_->init)
			{
				dualArm.setInputPos(angle1);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(angle1);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(angle1[i]);
				}

				if (motorsPositionCheck(current_angle, angle1, 12))
				{
					mout() << "Target 1 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_1);
					eeA2.getMpm(imp_->arm2_ee_pm_1);

					imp_->target1_reached = true;
					imp_->stop_count = 1;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}

			}
			else if (imp_->target1_reached && !imp_->target2_reached)
			{
				dualArm.setInputPos(angle2);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(angle2);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(angle2[i]);
				}

				if (motorsPositionCheck(current_angle, angle2, 12))
				{
					mout() << "Target 2 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_2);
					eeA2.getMpm(imp_->arm2_ee_pm_2);

					imp_->target2_reached = true;
					imp_->stop_count = 2;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;
				}
			}
			else if (imp_->target2_reached && !imp_->target3_reached)
			{
				dualArm.setInputPos(angle3);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(angle3);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(angle3[i]);
				}

				if (motorsPositionCheck(current_angle, angle3, 12))
				{
					mout() << "Target 3 Reached" << std::endl;

					eeA1.getMpm(imp_->arm1_ee_pm_3);
					eeA2.getMpm(imp_->arm2_ee_pm_3);

					imp_->target3_reached = true;
					imp_->stop_count = 3;
					imp_->current_stop_time = count();
					imp_->stop_flag = true;
					mout() << "current stop time: " << imp_->current_stop_time << std::endl;

				}
			}
			else if (imp_->target3_reached && !imp_->target4_reached)
			{
				// Back To Init
				dualArm.setInputPos(init_angle);
				if (dualArm.forwardKinematics())std::cout << "forward failed" << std::endl;

				//daJointMove(init_angle);

				for (std::size_t i = 0; i < 12; ++i)
				{
					controller()->motorPool()[i].setTargetPos(init_angle[i]);
				}

				if (motorsPositionCheck(current_angle, init_angle, 12))
				{
					mout() << "Back To Init Pos" << std::endl;
					imp_->target4_reached = true;

				}

			}
			else if (imp_->target1_reached && imp_->target2_reached && imp_->target3_reached && imp_->target4_reached)
			{





				//Arm 1
				double arm1_t_vector[9]{ 0 };
				double arm1_f_vector[9]{ 0 };

				double arm1_f_matrix[54]{ 0 };
				double arm1_r_matrix[54]{ 0 };


				double arm1_ee_rm_1[9]{ 0 };
				double arm1_ee_rm_2[9]{ 0 };
				double arm1_ee_rm_3[9]{ 0 };

				double arm1_current_force[6]{ 0 };

				aris::dynamic::s_pm2rm(imp_->arm1_ee_pm_1, arm1_ee_rm_1);
				aris::dynamic::s_pm2rm(imp_->arm1_ee_pm_2, arm1_ee_rm_2);
				aris::dynamic::s_pm2rm(imp_->arm1_ee_pm_3, arm1_ee_rm_3);



				gc.getTorqueVector(imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3, arm1_t_vector);
				gc.getForceVector(imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3, arm1_f_vector);

				gc.getFMatrix(imp_->arm1_force_data_1, imp_->arm1_force_data_2, imp_->arm1_force_data_3, arm1_f_matrix);
				gc.getRMatrix(arm1_ee_rm_1, arm1_ee_rm_2, arm1_ee_rm_3, arm1_r_matrix);

				gc.getPLMatrix(arm1_f_matrix, arm1_t_vector, imp_->arm1_p_vector);
				gc.getPLMatrix(arm1_r_matrix, arm1_f_vector, imp_->arm1_l_vector);

				double arm1_current_ee_pm[16]{ 0 };
				eeA1.getMpm(arm1_current_ee_pm);

				gc.getCompFT(arm1_current_ee_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, imp_->arm1_comp_f);


				//getForceData(arm1_current_force, 0, imp_->init);


				//Arm 2
				double arm2_t_vector[9]{ 0 };
				double arm2_f_vector[9]{ 0 };

				double arm2_f_matrix[54]{ 0 };
				double arm2_r_matrix[54]{ 0 };


				double arm2_ee_rm_1[9]{ 0 };
				double arm2_ee_rm_2[9]{ 0 };
				double arm2_ee_rm_3[9]{ 0 };

				double arm2_current_force[6]{ 0 };

				aris::dynamic::s_pm2rm(imp_->arm2_ee_pm_1, arm2_ee_rm_1);
				aris::dynamic::s_pm2rm(imp_->arm2_ee_pm_2, arm2_ee_rm_2);
				aris::dynamic::s_pm2rm(imp_->arm2_ee_pm_3, arm2_ee_rm_3);



				gc.getTorqueVector(imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3, arm2_t_vector);
				gc.getForceVector(imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3, arm2_f_vector);

				gc.getFMatrix(imp_->arm2_force_data_1, imp_->arm2_force_data_2, imp_->arm2_force_data_3, arm2_f_matrix);
				gc.getRMatrix(arm2_ee_rm_1, arm2_ee_rm_2, arm2_ee_rm_3, arm2_r_matrix);

				gc.getPLMatrix(arm2_f_matrix, arm2_t_vector, imp_->arm2_p_vector);
				gc.getPLMatrix(arm2_r_matrix, arm2_f_vector, imp_->arm2_l_vector);

				double arm2_current_ee_pm[16]{ 0 };
				eeA2.getMpm(arm2_current_ee_pm);

				gc.getCompFT(arm2_current_ee_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, imp_->arm2_comp_f);

				//getForceData(arm2_current_force, 1, imp_->init);

				gc.savePLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
				gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);


				mout() << "Current Arm1 Force After Compensation:" << '\n' << arm1_current_force[0] + imp_->arm1_comp_f[0] << '\t' << arm1_current_force[1] + imp_->arm1_comp_f[1] << '\t'
					<< arm1_current_force[2] + imp_->arm1_comp_f[2] << '\t' << arm1_current_force[3] + imp_->arm1_comp_f[3] << '\t'
					<< arm1_current_force[4] + imp_->arm1_comp_f[4] << '\t' << arm1_current_force[5] + imp_->arm1_comp_f[5] << std::endl;

				//mout() << "Current Arm1 End Pos:" << '\n' << arm1_current_ee_pm[0] << '\t' << arm1_current_ee_pm[1] << '\t'
				//	<< arm1_current_ee_pm[2] << '\t' << arm1_current_ee_pm[3] << '\t'
				//	<< arm1_current_ee_pm[4] << '\t' << arm1_current_ee_pm[5] << std::endl;

				mout() << "Current Arm2 Force After Compensation:" << '\n' << arm2_current_force[0] + imp_->arm2_comp_f[0] << '\t' << arm2_current_force[1] + imp_->arm2_comp_f[1] << '\t'
					<< arm2_current_force[2] + imp_->arm2_comp_f[2] << '\t' << arm2_current_force[3] + imp_->arm2_comp_f[3] << '\t'
					<< arm2_current_force[4] + imp_->arm2_comp_f[4] << '\t' << arm2_current_force[5] + imp_->arm2_comp_f[5] << std::endl;

				//mout() << "Current Arm2 End Pos:" << '\n' << arm2_current_ee_pm[0] << '\t' << arm2_current_ee_pm[1] << '\t'
				//	<< arm2_current_ee_pm[2] << '\t' << arm2_current_ee_pm[3] << '\t'
				//	<< arm2_current_ee_pm[4] << '\t' << arm2_current_ee_pm[5] << std::endl;

				

				return 0;

			}

			if (count() == 100000)
			{


				mout() << "Over Time" << std::endl;


			}

			return 100000 - count();
			
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



	struct ForceAlign::Imp {

			bool init = false;
			bool contact_check = false;

			double comp_f[6]{ 0 };

			double arm1_init_force[6]{ 0 };
			double arm2_init_force[6]{ 0 };

			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			double x_d;

			double Ke = 220000;
			double K = 3;

			double B[6]{ 0.25,0.25,0.7,0.0,0.0,0.0 };
			double M[6]{ 0.1,0.1,0.1,0.1,0.1,0.1 };

			double desired_force = -5;
			int contact_count;

			//Simulation
			double actual_force[6]{ 0 };

			//Switch Model
			int m_;

		};
	auto ForceAlign::prepareNrt()->void
	{

		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;




	}
	auto ForceAlign::executeRT()->int
	{

		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, PI / 2 };

		imp_->m_ = int32Param("model");

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


		GravComp gc;



		double current_vel[6]{ 0 };
		double current_pos[6]{ 0 };

		double current_force[6]{ 0 };
		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };
		double current_pm[16]{ 0 };

		double comp_force[6]{ 0 };
		double actual_force[6]{ 0 };

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}

			}

			if (m_ == 0)
			{
				//				data_[0] = -data_[0];
				//				data_[1] = -data_[1];

				//				data_[3] = -data_[3];
				//				data_[4] = -data_[4];
			}
			else if (m_ == 1)
			{
				data_[0] = -data_[0];
				data_[1] = -data_[1];

				data_[3] = -data_[3];
				data_[4] = -data_[4];

			}
			else
			{
				mout() << "Wrong Model" << std::endl;
			}


		};


		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};

		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
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


		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (!imp_->init)
		{
			dualArm.setInputPos(init_angle);
			if (dualArm.forwardKinematics())
			{
				mout() << "forward fail" << std::endl;
			}

			daJointMove(init_angle);
			if (motorsPositionCheck(current_angle, init_angle, 12))
			{
				//getForceData(imp_->arm1_init_force, 0, imp_->init);
				//getForceData(imp_->arm2_init_force, 1, imp_->init);
				mout() << "Back To Init" << std::endl;

				imp_->init = true;
			}
		}
		else
		{
			//Arm1
			if (imp_->m_ == 0)
			{
				//Update Statue
				eeA1.getV(current_vel);
				eeA1.getP(current_pos);
				eeA1.getMpm(current_pm);

				std::copy(current_angle, current_angle + 6, current_sa_angle);

				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}



				//Contact Check

				//test
				if (count() > 28000 && count() <= 29000)
				{
					imp_->actual_force[2] = -0.55;
				}
				else if (count() > 29000 && count() <= 33000)
				{
					imp_->actual_force[2] = -5;
				}
				else if (count() > 33000)
				{
					imp_->actual_force[2] = -7;
				}


				//Get Actual Force
				//getForceData(current_force, imp_->m_, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				if (!imp_->contact_check)
				{
					if (abs(imp_->actual_force[2]) > 2)
					{
						imp_->contact_check = true;
						imp_->contact_count = count();
						// Set Disred Pos 
						imp_->x_d = current_pos[0];
						mout() << "Contacted! Disred X Pos: " << imp_->x_d << std::endl;
					}
					else
					{

						current_pos[0] -= 0.00003;
						saMove(current_pos, model_a1, 0);

					}

				}
				else if (imp_->contact_check)
				{
					double x_r = imp_->x_d - (imp_->desired_force / imp_->Ke);
					double a = (imp_->desired_force - imp_->actual_force[2] - imp_->B[2] * current_vel[0] - imp_->K * (current_pos[0] - x_r)) / imp_->M[2];
					double x = current_pos[0] + 0.5 * a * 0.001 * 0.001 + current_vel[0] * 0.001;

					current_pos[0] = x;
					saMove(current_pos, model_a1, 0);

					//if (count() % 100 == 0)
					//{
					//	//mout() << "count(): " << count() << std::endl;
					//	mout() << "force: " << imp_->actual_force[0] << '\t' << imp_->actual_force[1] << '\t' << imp_->actual_force[2] << '\t'
					//		<< imp_->actual_force[3] << '\t' << imp_->actual_force[4] << '\t' << imp_->actual_force[5] << std::endl;

					//}

				}
			}
			//Arm2
			else if (imp_->m_ == 1)
			{
				//Update Statue
				eeA2.getV(current_vel);
				eeA2.getP(current_pos);
				eeA2.getMpm(current_pm);

				std::copy(current_angle + 6, current_angle + 12, current_sa_angle);
				

				if (count() % 100 == 0)
				{
					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
				}

				//test
				if (count() > 28000 && count() <= 29000)
				{
					imp_->actual_force[2] = -0.55;
				}
				else if (count() > 29000 && count() <= 33000)
				{
					imp_->actual_force[2] = -5;
				}
				else if (count() > 33000)
				{
					imp_->actual_force[2] = -7;
				}

				//Contact Check

				//Get Actual Force
				//getForceData(current_force, imp_->m_, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}

				if (!imp_->contact_check)
				{
					if (abs(imp_->actual_force[2]) > 2)
					{
						imp_->contact_check = true;
						imp_->contact_count = count();
						// Set Disred Pos 
						imp_->x_d = current_pos[0];
						mout() << "Contacted! Disred X Pos: " << imp_->x_d << std::endl;
					}
					else
					{

						current_pos[0] -= 0.00003;
						saMove(current_pos, model_a2, 1);

					}

				}
				else if (imp_->contact_check)
				{
					double x_r = imp_->x_d - (imp_->desired_force / imp_->Ke);
					double a = (imp_->desired_force - imp_->actual_force[2] - imp_->B[2] * current_vel[0] - imp_->K * (current_pos[0] - x_r)) / imp_->M[2];
					double x = current_pos[0] + 0.5 * a * 0.001 * 0.001 + current_vel[0] * 0.001;

					current_pos[0] = x;
					saMove(current_pos, model_a2, 1);

					if (count() % 1000 == 0)
					{
						//mout() << "count(): " << count() << std::endl;
						mout() << "force: " << imp_->actual_force[0] << '\t' << imp_->actual_force[1] << '\t' << imp_->actual_force[2] << '\t'
							<< imp_->actual_force[3] << '\t' << imp_->actual_force[4] << '\t' << imp_->actual_force[5] << std::endl;

					}

				}
			}
			//Error
			else 
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}
			
		}
		//Over Time Exit
		if (count() == 800000)
		{
			mout() << "Over Time" << std::endl;
		}

		return 800000 - count();

	}	
	ForceAlign::ForceAlign(const std::string& name): imp_(new Imp)
	{

		aris::core::fromXmlString(command(),
			"<Command name=\"m_fa\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceAlign::~ForceAlign() = default;




	struct ForceKeep::Imp {

			//Flag
			bool init = false;
			bool contact_check = false;

			//Force Compensation Parameter
			double comp_f[6]{ 0 };

			//Arm1
			double arm1_init_force[6]{ 0 };
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			//Arm2
			double arm2_init_force[6]{ 0 };
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			//Desired Pos, Vel, Acc, Foc
			double arm1_x_d[6]{ 0 };
			double arm2_x_d[6]{ 0 };

			double v_d[6]{ 0 };
			double a_d[6]{ 0 };
			double f_d[6]{ 0 };

			//Current Vel
			double v_c[6]{ 0 };

			//Impedence Parameter
			double a1_K[6]{ 100,100,100,5,5,5 };
			double a1_B[6]{ 100,100,100,5,5,5 };
			double a1_M[6]{ 1,1,1,2,2,2 };

			double a2_K[6]{ 100,100,100,15,15,15 };
			double a2_B[6]{ 100,100,100,15,15,15 };
			double a2_M[6]{ 1,1,1,10,10,10 };

			double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

			//Counter
			int contact_count = 0;

			//Test
			double actual_force[6]{ 0 };

			//Switch Model
			int m_;

			//Force Buffer
			std::array<double, 10> force_buffer[6] = {};
			int buffer_index[6]{ 0 };
		};
	auto ForceKeep::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceKeep::executeRT() -> int
	{
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

		//ver 1.0 not limit on vel, only limit force
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
		static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
		static double max_force[6]{ 10,10,10,5,5,5 };
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		GravComp gc;

		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };

		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double filtered_force[6]{ 0 };
		double transform_force[6]{ 0 };

		imp_->m_ = int32Param("model");

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}
			}
		};


		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
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

		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
			}
		};


		for (std::size_t i = 0; i < 6; ++i)
		{
			imp_->actual_force[i] = 0;
		}



		if (count() > 2000 && count() <= 4000)
		{
			//mout() << "fex" << std::endl;
			imp_->actual_force[3] = 0.5;
		}
		//else if (count() > 9000 && count() <= 10000)
		//{
		//	//mout() << "fex" << std::endl;
		//	imp_->actual_force[0] = 5;
		//}


		 //else if (count() > 2000 && count() <= 3000)
		 //{
		 //	imp_->actual_force[2] = -10;
		 //	imp_->actual_force[0] = -15;
		 //}
		 //else if (count() > 3000 && count() <= 4000)
		 //{
		 //	//imp_->actual_force[1] = -5;
		 //	imp_->actual_force[4] = -6;
		 //}




		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (!imp_->init && !imp_->contact_check)
		{



			dualArm.setInputPos(init_angle);

			if (dualArm.forwardKinematics())
			{
				throw std::runtime_error("Forward Kinematics Position Failed!");
			}

			//Test
			//for (std::size_t i = 0; i < 12; ++i)
			//{
			//	controller()->motorPool()[i].setTargetPos(init_angle[i]);
			//}

			daJointMove(init_angle);

			//if (count() % 1000 == 0)
			//{

			//	mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t'
			//		<< current_angle[3] << '\t' << current_angle[4] << '\t' << current_angle[5] << std::endl;

			//}

			if (motorsPositionCheck(current_angle, init_angle, 12))
			{

				eeA1.getP(imp_->arm1_x_d);
				eeA2.getP(imp_->arm2_x_d);

				mout() << imp_->arm1_x_d[0] << '\t' << imp_->arm1_x_d[1] << '\t' << imp_->arm1_x_d[2] << '\t'
					<< imp_->arm1_x_d[3] << '\t' << imp_->arm1_x_d[4] << '\t' << imp_->arm1_x_d[5] << std::endl;

				//getForceData(imp_->arm1_init_force, 0, imp_->init);
				//getForceData(imp_->arm2_init_force, 1, imp_->init);

				mout() << "Back To Init" << std::endl;
				imp_->init = true;
			}


		}
		else if (imp_->init && !imp_->contact_check)
		{
			double raw_force_checker[12]{ 0 };
			double comp_force_checker[12]{ 0 };
			double force_checker[12]{ 0 };

			double a1_pm[16]{ 0 };
			double a2_pm[16]{ 0 };

			eeA1.getMpm(a1_pm);
			eeA2.getMpm(a2_pm);


			//Arm1
			getForceData(raw_force_checker, 0, imp_->init);
			gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
			//Arm2
			getForceData(raw_force_checker + 6, 1, imp_->init);
			gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);

			
			for (int i = 0; i < 12; i++)
			{
				force_checker[i] = comp_force_checker[i] + raw_force_checker[i];

				if (abs(force_checker[i]) > 3) 
				{
					imp_->contact_check = true;
					mout() << "Contact Check" << std::endl;
					break;
				}
			}
		}
		else
		{
			if (imp_->m_ == 0)
			{
				double current_vel[6]{ 0 };
				eeA1.getP(current_pos);
				eeA1.getV(current_vel);
				eeA1.getMpm(current_pm);

				getForceData(current_force, 0, imp_->init);
				gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				for (int i = 0; i < 6; i++)
				{
					actual_force[i] = comp_force[i] + current_force[i];
				}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

				forceFilter(actual_force, filtered_force);

				//Coordinate Transform Arm1
				transform_force[0] = filtered_force[2];
				transform_force[1] = -filtered_force[1];
				transform_force[2] = filtered_force[0];

				transform_force[3] = filtered_force[5];
				transform_force[4] = -filtered_force[4];
				transform_force[5] = filtered_force[3];


				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->a1_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a1_K[i] * (current_pos[i] - imp_->arm1_x_d[i])) / imp_->a1_M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}

				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };


				//Rm of Desired Pos
				aris::dynamic::s_re2rm(imp_->arm1_x_d + 3, rm_d, "321");

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Inverse
				aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);

				//Convert Rm to Ra
				aris::dynamic::s_rm2ra(rm_e, pose_error);



				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->a1_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a1_K[i + 3] * pose_error[i]) / imp_->a1_M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };


				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");


				eeA1.setV(imp_->v_c);
				if (model_a1.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a1, 0);




				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					// mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
					// 	<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

					mout() << "error: " << pose_error[0] << '\t' << pose_error[1] << '\t' << pose_error[2] << std::endl;

				}

			}
			else if (imp_->m_ == 1)
			{

				double current_vel[6]{ 0 };

				eeA2.getP(current_pos);

				eeA2.getV(current_vel);
				eeA2.getMpm(current_pm);

				//getForceData(current_force, 1, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}

				//Dead Zone of Force
				for (int i = 0; i < 6; i++)
				{
					if (abs(actual_force[i]) < trigger_force[i])
					{
						actual_force[i] = 0;
					}
					if (actual_force[i] > max_force[i])
					{
						actual_force[i] = max_force[i];
					}
					if (actual_force[i] < -max_force[i])
					{
						actual_force[i] = -max_force[i];
					}

				}

				forceFilter(actual_force, filtered_force);

				//Coordinate Transform Arm2
				transform_force[0] = -filtered_force[2];
				transform_force[1] = filtered_force[1];
				transform_force[2] = filtered_force[0];

				transform_force[3] = -filtered_force[5];
				transform_force[4] = filtered_force[4];
				transform_force[5] = filtered_force[3];

				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					//acc[i] = (imp_->f_d[i] - imp_->actual_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->K[i] * (current_pos[i] - imp_->arm2_x_d[i])) / imp_->M[i];

					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->a2_B[i] * (imp_->v_c[i] - imp_->v_d[i]) - imp_->a2_K[i] * (current_pos[i] - imp_->arm2_x_d[i])) / imp_->a2_M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


				double rm_c[9]{ 0 };
				double rm_d[9]{ 0 };
				double rm_e[9]{ 0 };
				double pose_error[3]{ 0 };


				//Rm of Desired Pos
				aris::dynamic::s_re2rm(imp_->arm2_x_d + 3, rm_d, "321");

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Inverse
				aris::dynamic::s_rm_dot_inv_rm(rm_c, rm_d, rm_e);


				//Convert Rm to Ra
				aris::dynamic::s_rm2ra(rm_e, pose_error);

			


				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->a2_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3]) - imp_->a2_K[i + 3] * pose_error[i]) / imp_->a2_M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };


				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

				eeA2.setV(imp_->v_c);
				if (model_a2.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a2, 1);



				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				}
			}
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}





		}

		return 30000 - count();
	}
	ForceKeep::ForceKeep(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fk\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceKeep::~ForceKeep() = default;



	struct ForceDrag::Imp {

			//Flag
			bool init = false;
			bool contact_check = false;

			//Force Compensation Parameter
			double comp_f[6]{ 0 };

			//Arm1
			double arm1_init_force[6]{ 0 };
			double arm1_p_vector[6]{ 0 };
			double arm1_l_vector[6]{ 0 };

			//Arm2
			double arm2_init_force[6]{ 0 };
			double arm2_p_vector[6]{ 0 };
			double arm2_l_vector[6]{ 0 };

			//Desired Pos, Vel, Acc, Foc
			double arm1_x_d[6]{ 0 };
			double arm2_x_d[6]{ 0 };

			double v_d[6]{ 0 };
			double a_d[6]{ 0 };
			double f_d[6]{ 0 };

			//Current Vel
			double v_c[6]{ 0 };

			//Impedence Parameter
			//double K[6]{ 100,100,100,100,100,100 };
			double B[6]{ 100,100,100,5,5,2 };
			double M[6]{ 100,100,100,5,5,2 };

			double Ke[6]{ 220000,220000,220000,220000,220000,220000 };

			//Counter
			int contact_count = 0;

			//Test
			double actual_force[6]{ 0 };

			//Switch Model
			int m_;

			//Force Buffer
			std::array<double, 10> force_buffer[6] = {};
			int buffer_index[6]{ 0 };
		};
	auto ForceDrag::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto ForceDrag::executeRT() -> int
	{
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

		//ver 1.0 not limit on vel, only limit force
		static double tolerance = 0.0001;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		static double max_vel[6]{ 0.2,0.2,0.2,0.0005,0.0005,0.0001 };
		static double trigger_force[6]{ 0.5,0.5,0.5,0.001,0.001,0.001 };
		static double max_force[6]{ 10,10,10,5,5,5 };
		static double trigger_vel[6]{ 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001 };

		GravComp gc;

		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };

		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double filtered_force[6]{ 0 };
		double transform_force[6]{ 0 };

		imp_->m_ = int32Param("model");

		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}

			}

		};


		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};


		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
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


		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
			}
		};


		//for (std::size_t i = 0; i < 6; ++i)
		//{
		//	imp_->actual_force[i] = 0;
		//}



		//if (count() > 5000 && count() <= 7000)
		//{
		//	//mout() << "fex" << std::endl;
		//	imp_->actual_force[0] = -5;
		//}
		//else if (count() > 9000 && count() <= 10000)
		//{
		//	//mout() << "fex" << std::endl;
		//	imp_->actual_force[0] = 5;
		//}


		 //else if (count() > 2000 && count() <= 3000)
		 //{
		 //	imp_->actual_force[2] = -10;
		 //	imp_->actual_force[0] = -15;
		 //}
		 //else if (count() > 3000 && count() <= 4000)
		 //{
		 //	//imp_->actual_force[1] = -5;
		 //	imp_->actual_force[4] = -6;
		 //}




		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}



		if (!imp_->init && !imp_->contact_check)
		{



			dualArm.setInputPos(init_angle);

			if (dualArm.forwardKinematics())
			{
				throw std::runtime_error("Forward Kinematics Position Failed!");
			}

			//Test
			for (std::size_t i = 0; i < 12; ++i)
			{
				controller()->motorPool()[i].setTargetPos(init_angle[i]);
			}

			daJointMove(init_angle);

			//if (count() % 1000 == 0)
			//{

			//	mout() << current_angle[0] << '\t' << current_angle[1] << '\t' << current_angle[2] << '\t'
			//		<< current_angle[3] << '\t' << current_angle[4] << '\t' << current_angle[5] << std::endl;

			//}

			if (motorsPositionCheck(current_angle, init_angle, 12))
			{

				eeA1.getP(imp_->arm1_x_d);
				eeA2.getP(imp_->arm2_x_d);

				mout() << imp_->arm1_x_d[0] << '\t' << imp_->arm1_x_d[1] << '\t' << imp_->arm1_x_d[2] << '\t'
					<< imp_->arm1_x_d[3] << '\t' << imp_->arm1_x_d[4] << '\t' << imp_->arm1_x_d[5] << std::endl;

				//getForceData(imp_->arm1_init_force, 0, imp_->init);
				//getForceData(imp_->arm2_init_force, 1, imp_->init);

				mout() << "Back To Init" << std::endl;
				imp_->init = true;
			}


		}
		else if (imp_->init && !imp_->contact_check)
		{
			double raw_force_checker[12]{ 0 };
			double comp_force_checker[12]{ 0 };
			double force_checker[12]{ 0 };

			double a1_pm[16]{ 0 };
			double a2_pm[16]{ 0 };

			eeA1.getMpm(a1_pm);
			eeA2.getMpm(a2_pm);


			//Arm1
			getForceData(raw_force_checker, 0, imp_->init);
			gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
			//Arm2
			getForceData(raw_force_checker + 6, 1, imp_->init);
			gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker + 6);


			for (int i = 0; i < 12; i++)
			{
				force_checker[i] = comp_force_checker[i] + raw_force_checker[i];

				if (abs(force_checker[i]) > 3)
				{
					imp_->contact_check = true;
					mout() << "Contact Check" << std::endl;
					break;
				}
			}
		}
		else
		{
			if (imp_->m_ == 0)
			{
				double current_vel[6]{ 0 };

				eeA1.getP(current_pos);
				eeA1.getV(current_vel);
				eeA1.getMpm(current_pm);

				//getForceData(current_force, 0, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}

				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;

				forceFilter(actual_force, filtered_force);

				//Coordinate Transform Arm1
				transform_force[0] = filtered_force[2];
				transform_force[1] = -filtered_force[1];
				transform_force[2] = filtered_force[0];

				transform_force[3] = filtered_force[5];
				transform_force[4] = -filtered_force[4];
				transform_force[5] = filtered_force[3];

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };
				double rm_c[9]{ 0 };




				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

				eeA1.setV(imp_->v_c);
				if (model_a1.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a1, 0);


				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				}

			}
			else if (imp_->m_ == 1)
			{

				double current_vel[6]{ 0 };

				eeA2.getP(current_pos);

				eeA2.getV(current_vel);
				eeA2.getMpm(current_pm);

				//getForceData(current_force, 1, imp_->init);
				//gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
				//for (int i = 0; i < 6; i++)
				//{
				//	actual_force[i] = comp_force[i] + current_force[i];
				//}

				double acc[3]{ 0 };
				double ome[3]{ 0 };
				double pm[16]{ 0 };
				double dx[3]{ 0 };
				double dth[3]{ 0 };
				double dt = 0.001;

				forceFilter(actual_force, filtered_force);

				//Coordinate Transform Arm2
				transform_force[0] = -filtered_force[2];
				transform_force[1] = filtered_force[1];
				transform_force[2] = filtered_force[0];

				transform_force[3] = -filtered_force[5];
				transform_force[4] = filtered_force[4];
				transform_force[5] = filtered_force[3];

				//position
				for (int i = 0; i < 3; i++)
				{
					// da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
					acc[i] = (-imp_->f_d[i] + imp_->actual_force[i] - imp_->B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->M[i];
				}


				for (int i = 0; i < 3; i++)
				{
					imp_->v_c[i] += acc[i] * dt;
					dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
					current_pos[i] = dx[i] + current_pos[i];

				}


				//pose
				for (int i = 0; i < 3; i++)
				{
					// Caculate Omega
					ome[i] = (-imp_->f_d[i + 3] + imp_->actual_force[i + 3] - imp_->B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->M[i + 3];
				}


				for (int i = 0; i < 3; i++)
				{
					// Angluar Velocity
					imp_->v_c[i + 3] += ome[i] * dt;
					dth[i] = imp_->v_c[i + 3] * dt;
				}

				double drm[9]{ 0 };
				double rm_target[9]{ 0 };
				double rm_c[9]{ 0 };




				//Transform to rm
				aris::dynamic::s_ra2rm(dth, drm);

				//Current pe to rm
				aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

				//Calcuate Future rm
				aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

				//Convert rm to pe
				aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");


				//mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
				//	<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				//mout() << dx[0] << '\t' << dx[1] << '\t' << dx[2] << '\t'
				//	<< dx[3] << '\t' << dx[4] << '\t' << dx[5] << std::endl;

				//mout() << acc[0] << '\t' << acc[1] << '\t' << acc[2] << '\t'
				//	<< acc[3] << '\t' << acc[4] << '\t' << acc[5] << std::endl;

				eeA2.setV(imp_->v_c);
				if (model_a2.inverseKinematicsVel()) {
					mout() << "Error" << std::endl;
				}
				saMove(current_pos, model_a2, 1);



				if (count() % 100 == 0)
				{
					// mout() << current_vel[0] << '\t' << current_vel[1] << '\t' << current_vel[2] << '\t'
					// 	<< current_vel[3] << '\t' << current_vel[4] << '\t' << current_vel[5] << std::endl;

					mout() << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
						<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

				}
			}
			else
			{
				mout() << "Wrong Model" << std::endl;
				return 0;
			}





		}

		return 10000 - count();
	}
	ForceDrag::ForceDrag(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"m_fd\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	ForceDrag::~ForceDrag() = default;




	struct PegInHole::Imp {

		//Flag
		//Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
		bool init = false;
		bool stop = false;
		bool phase1 = false;
		bool phase2 = false;
		bool phase3 = false;
		bool phase4 = false;
		bool phase5 = false;
        bool phase6 = false;

		//Force Compensation Parameter
		double comp_f[6]{ 0 };

		//Arm1
		double arm1_init_force[6]{ 0 };
		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };

		//Arm2
		double arm2_init_force[6]{ 0 };
		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };

		//Desired Pos, Vel, Acc, Foc
		double arm1_x_d[6]{ 0 };
		double arm2_x_d[6]{ 0 };

		double v_d[6]{ 0 };
		double a_d[6]{ 0 };
		double f_d[6]{ 0 };

		//Desired Force of Each Phase
		double phase2_fd[6]{ 0 };
        double phase3_fd[6]{ -4.5,0,0,0,0,0 };
        double phase4_fd[6]{ -3.5,0,0,0,0,0 };
        double phase5_fd[6]{ -5.0,0,0,0,0,0 };
        double phase6_fd[6]{ 0,0,0,0,0,0 };

		//Desired Pos of Each Phase
		double phase2_xd[6]{ 0 };

		//Current Vel
		double v_c[6]{ 0 };

		//Impedence Parameter
        double phase3_B[6]{ 1500,1500,1500,0,0,0 };
        double phase3_M[6]{ 100,100,100,0, 0, 0 };

        double phase4_B[6]{ 3000,3300,3300,4.5,4.5,4.5 };
        double phase4_M[6]{ 150,100,100,2.5,2.5,2.5 };

        double phase5_B[6]{ 2000,3500,3500,450,450,450 };
        double phase5_M[6]{ 100,200,200,200, 200, 200 };

        double phase6_B[6]{ 0,5000,5000,0,0,0 };
        double phase6_M[6]{ 0,100,100,0, 0, 0 };

		//Counter
        int contact_count = 0;
        int current_count = 0;

        //Allign Counter
        int allign_count = 0;
        int allign_success_count = 0;


        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        double current_pos_checkek[6] = {0};

		//Test
		//double actual_force[6]{ 0 };

		//Switch Model
		int m_;

		//Force Buffer
        std::array<double, 20> force_buffer[6] = {};
		int buffer_index[6]{ 0 };
	};
	auto PegInHole::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		GravComp gc;
		gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		mout() << "Load P & L Vector" << std::endl;
	}
	auto PegInHole::executeRT() -> int
	{
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

		//ver 1.0 not limit on vel, only limit force
		static double tolerance = 0.00005;
		static double init_angle[12] =
		{ 0, 0, 5 * PI / 6, -5 * PI / 6, -PI / 2, 0 ,
		0, 0, -5 * PI / 6, 5 * PI / 6, PI / 2, 0 };
		//ver 2.0 limited vel, 20mm/s, 30deg/s
		static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };

        static double trigger_force[6]{ 0.5,0.5,0.5,0.0015,0.0015,0.0015 };

		static double trigger_vel[6]{ 0.000001,0.000001,0.000001,0.0001,0.0001,0.0001 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0015,0.011,0.0015 };
        static double limit_area[6]{17,8,8,0.6,0.6,0.6};

        double p3_deadzone[6]{0,10,10,0,0,0};
        double p4_deadzone[6]{0,0,0,0.2,0.2,0.2};
        double p5_deadzone[6]{0,0,0,0.1,0.1,0.1};


		GravComp gc;

		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };

		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double filtered_force[6]{ 0 };
		double transform_force[6]{ 0 };
        double limited_force[6]{ 0 };


		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}

			}

		};


		auto daJointMove = [&](double target_mp_[12])
		{
			double current_angle[12] = { 0 };
			double move = 0.00005;

			for (int i = 0; i < 12; i++)
			{
				current_angle[i] = controller()->motorPool()[i].targetPos();
			}

			for (int i = 0; i < 12; i++)
			{
				if (current_angle[i] <= target_mp_[i] - move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
				}
				else if (current_angle[i] >= target_mp_[i] + move)
				{
					controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
				}
			}
		};

		auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
		{
			for (int i = 0; i < dim_; i++)
			{
				if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
				{
					return false;
				}
			}

			return true;
		};

		//single arm move 1-->white 2-->blue
        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };



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


		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
                imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 20;

                filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 20;
			}
		};

		auto forceDeadZone = [&](double* actual_force_, double* area_)
		{
			for (int i = 0; i < 6; i++)
			{
				if (abs(actual_force_[i]) < area_[i])
				{
					actual_force_[i] = 0;
				}
			}
		};

        auto forceUpperLimit = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (actual_force_[i] >= area_[i])
                {
                    actual_force_[i] = area_[i];
                }
                else if(actual_force_[i] <= -area_[i])
                {
                    actual_force_[i] = -area_[i];
                }
            }
        };

		auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
		{
			if (m_ == 0)
			{
				transform_force_[0] = actual_force_[2];
				transform_force_[1] = -actual_force_[1];
				transform_force_[2] = actual_force_[0];

				transform_force_[3] = actual_force_[5];
				transform_force_[4] = -actual_force_[4];
				transform_force_[5] = actual_force_[3];
			}
			else if (m_ == 1)
			{
				transform_force_[0] = -actual_force_[2];
				transform_force_[1] = actual_force_[1];
				transform_force_[2] = actual_force_[0];

				transform_force_[3] = -actual_force_[5];
				transform_force_[4] = actual_force_[4];
				transform_force_[5] = actual_force_[3];
			}
			else
			{
				mout() << "Error Model In Force Transform" << std::endl;
			}
		};

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_)
        {

            bool isValid = true;
            for (int i = 0; i < 3; i++)
            {
                if (abs(current_force_[i]) > force_check_[i] + 0.05)
                {
                    isValid = false;
                    break;
                }
            }

            if (isValid)
            {
                if (imp_->allign_count == 0) {
                    imp_->allign_count = count();
                }

                if ((count() - imp_->allign_count) % 30 == 0) {
                    imp_->allign_success_count++;
                    mout() << "Check " << imp_->allign_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if (imp_->allign_success_count >= count_)
                    {
                        //Restore Counter
                        imp_->allign_count = 0;
                        imp_->allign_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->allign_success_count = 0;
            }

            return false;
        };

        auto posCheck = [&](double* current_pos_, int count_)
        {
            if(count()%500 == 0)
            {
                std::copy(current_pos_, current_pos_+6, imp_->current_pos_checkek);
                //mout()<<"Save Current Pos: "<<imp_->current_pos_checkek[0]<<std::endl;
            }

            bool isValid = true;

            for(int i = 0; i < 3; i++)
            {
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.00001)
                {
                    isValid = false;
                    break;
                }
            }

            if(isValid)
            {
                if(imp_->pos_count == 0)
                {
                    imp_->pos_count = count();
                }

                if((count() - imp_->pos_count) % 50 == 0)
                {
                    imp_->pos_success_count ++;
                    //mout() << "Check " << imp_->pos_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if(imp_->pos_success_count >= count_)
                    {
                        imp_->pos_count = 0;
                        imp_->pos_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->pos_success_count = 0;
            }

            return false;
        };



		auto velDeadZone = [&](double vel_, double vel_limit_)
		{
			if (vel_ >= vel_limit_)
			{
				vel_ = vel_limit_;
			}
			else if (vel_ <= -vel_limit_)
			{
				vel_ = -vel_limit_;
			}

		};




		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}


		std::copy(current_angle, current_angle + 6, current_sa_angle);



        if(!imp_->init)
        {
            double assem_pos[6]{ 0.730, 0.039101, 0.291316, PI / 2, -PI / 2, PI / 2 };
            double init_angle[6]{0};

            model_a1.setOutputPos(assem_pos);
            if(model_a1.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }

            model_a1.getInputPos(init_angle);

            saJointMove(init_angle, 0);

            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 0, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
            }
        }
        else
        {

            eeA1.getP(current_pos);
            eeA1.getMpm(current_pm);



            //Force Comp, Filtered, Transform
            getForceData(actual_force, 0, imp_->init);
            gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
            for (size_t i = 0; i < 6; i++)
            {
                comp_force[i] = actual_force[i] + comp_force[i];
            }

            forceFilter(comp_force, filtered_force);
            forceDeadZone(filtered_force, dead_zone);
            forceTransform(filtered_force, transform_force, 0);

            std::copy(transform_force, transform_force+6, limited_force);

            forceUpperLimit(limited_force, limit_area);

            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                double assem_pos[6]{ 0.730, 0.039101, 0.291316, PI / 2, -PI / 2, PI / 2 };
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };

                //Define Initial Rotate Error
                double rotate_angle[3]{ 0,10 * 2 * PI / 360, 0 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };

                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");

                eeA1.setP(assem_pos);


                if (model_a1.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }

                model_a1.getInputPos(assem_angle);

                saJointMove(assem_angle, 0);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {
                    imp_->phase1 = true;
                    mout() << "Assembly Start !" << std::endl;
                }

            }
            //Phase 2 Contact
            else if (imp_->phase1 && !imp_->phase2)
            {
                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };

                double a1_pm[16]{ 0 };
                eeA1.getMpm(a1_pm);
                eeA1.getP(current_pos);

                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;
                }


                //Arm1
                getForceData(raw_force_checker, 0, imp_->init);
                gc.getCompFT(a1_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force_checker);
                if (count() % 1000 == 0)
                {
                    mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }

                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) > 1.0)

                    {
                        imp_->phase2 = true;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                            << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                        break;

                    }

                }
                if (!imp_->phase2)
                {
                    current_pos[0] += 0.00001;
                    saMove(current_pos, model_a1, 0);
                }

            }
            //Phase 3 Position Move Only, Maintain Two-Point Contact
            else if (imp_->phase2 && !imp_->phase3)
            {

                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0,0,0,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 30.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (posCheck(current_pos, 10))
                {
                    imp_->phase3 = true;
                    mout() << "Pos Move Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Complete force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;
                }
                else
                {

                    if(limited_force[0] >= (imp_->phase3_fd[0] - 8.0) && limited_force[0] <= (imp_->phase3_fd[0] + 1))
                    {
                        limited_force[0] = imp_->phase3_fd[0];
                    }

                    forceDeadZone(limited_force, p3_deadzone);


                    if (count() % 100 == 0)
                    {

                        mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' <<"limited : "<<limited_force[0]<< '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << std::endl;
                    }



                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + limited_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 4 Allign
            else if (imp_->phase3 && !imp_->phase4)
            {

                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0.2,0.2,0.2,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 30.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' << '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }


                if (forceCheck(transform_force, desired_force, 5))
                {
                    imp_->phase4 = true;
                    mout() << "Allign Complete" << std::endl;
                    mout() << "Allign Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Allign force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                }
                else
                {

                    if(limited_force[0] >= (imp_->phase3_fd[0] - 5.0) && limited_force[0] <= (imp_->phase3_fd[0] + 1.5))
                    {
                        limited_force[0] = imp_->phase3_fd[0];
                    }

                    forceDeadZone(limited_force, p4_deadzone);

//                    for(int i = 4; i<6; i++)
//                    {
//                        if(limited_force[i] > 0)
//                        {
//                            limited_force[i] -= 0.1;
//                        }
//                        else if(limited_force[i] < 0)
//                        {
//                            limited_force[i] += 0.1;
//                        }
//                    }


                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + limited_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    //pose
                    for (int i = 1; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase4_fd[i + 3] + limited_force[i + 3] - imp_->phase4_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase4_M[i + 3];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        velDeadZone(imp_->v_c[i + 3], max_vel[i + 3]);
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 5 Insert
            else if (imp_->phase4 && !imp_->phase5)
            {
                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;
                double desired_force[6]{ 0,0,0,0,0,0 };


                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 10)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase4: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (count() % 100 == 0)
                {
                    mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;
                    mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                }

                //Complete Check
                if (current_pos[0] >= 0.805)
                {
                    imp_->phase5 = true;
                    mout() << "Insert Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;

                }
                else
                {

                    forceDeadZone(transform_force, p5_deadzone);

                    //Impedence Controller
                    for (int i = 0; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase4_fd[i] + transform_force[i] - imp_->phase4_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase4_M[i];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }


                    //pose
                    for (int i = 0; i < 3; i++)
                    {
                        // Caculate Omega
                        ome[i] = (-imp_->phase4_fd[i + 3] + transform_force[i + 3] - imp_->phase4_B[i + 3] * (imp_->v_c[i + 3] - imp_->v_d[i + 3])) / imp_->phase4_M[i + 3];
                    }


                    for (int i = 0; i < 3; i++)
                    {
                        // Angluar Velocity
                        imp_->v_c[i + 3] += ome[i] * dt;
                        dth[i] = imp_->v_c[i + 3] * dt;
                    }

                    double drm[9]{ 0 };
                    double rm_target[9]{ 0 };
                    double rm_c[9]{ 0 };

                    //Transform to rm
                    aris::dynamic::s_ra2rm(dth, drm);

                    //Current pe to rm
                    aris::dynamic::s_re2rm(current_pos + 3, rm_c, "321");

                    //Calcuate Future rm
                    aris::dynamic::s_mm(3, 3, 3, drm, rm_c, rm_target);

                    //Convert rm to pe
                    aris::dynamic::s_rm2re(rm_target, current_pos + 3, "321");

                    saMove(current_pos, model_a1, 0);
                }
            }
            //Phase 6 Force to Zero
            else if(imp_->phase5 && !imp_->phase6)
            {
                double acc[3]{ 0 };
                double dx[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0.7,0.5,0.5,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(transform_force[i]) > 5.0)
                    {
                        mout() << "Emergency Brake" << std::endl;

                        mout() << "Brake force Phase3: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                        return 0;
                    }
                }

                if (forceCheck(transform_force, desired_force, 3))
                {
                    imp_->phase3 = true;
                    mout() << "Move Complete" << std::endl;
                    mout() << "Complete Pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
                        << current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
                    mout() << "Complete force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                        << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;

                    return 0;
                }
                else
                {


                    if (count() % 100 == 0)
                    {

                        mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
                            << transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << '\t' <<"limited : "<<limited_force[0]<< '\t' <<"  pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << std::endl;
                    }



                    //Impedence Controller
                    for (int i = 1; i < 3; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase6_fd[i] + limited_force[i] - imp_->phase6_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase6_M[i];
                    }


                    for (int i = 1; i < 3; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        current_pos[i] = dx[i] + current_pos[i];

                    }

                    saMove(current_pos, model_a1, 0);
                }
            }
        }


        return 150000 - count();
	}
	PegInHole::PegInHole(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"m_ph\"/>");
}
    PegInHole::~PegInHole() = default;


	struct HoleInPeg::Imp {

        //Flag
        //Phase 1 -> Approach, Phase 2 -> Contact, Phase 3 -> Align, Phase 4 -> Fit, Phase 5 -> Insert
        bool init = false;
        bool stop = false;
        bool phase1 = false;
        bool phase2 = false;
        bool phase3 = false;
        bool phase4 = false;
        bool phase5 = false;

        //Force Compensation Parameter
        double comp_f[6]{ 0 };

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_start_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_start_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

        //Desired Pos, Vel, Acc, Foc
        double arm1_x_d[6]{ 0 };
        double arm2_x_d[6]{ 0 };

        double v_d[6]{ 0 };
        double a_d[6]{ 0 };
        double f_d[6]{ 0 };

        //Desired Force of Each Phase
        double phase2_fd[6]{ 0 };
        double phase3_fd[6]{ 5,0,0,0,0,0 };
        double phase4_fd[6]{ -2.0,0,0,0,0,0 };
        double phase5_fd[6]{ 0 };

        //Desired Pos of Each Phase
        double phase2_xd[6]{ 0 };

        //Current Vel
        double v_c[6]{ 0 };

        //Impedence Parameter
        //double K[6]{ 100,100,100,15,15,15 };
        double phase3_B[6]{ 25000,1500,1500,3,3,3 };
        double phase3_M[6]{ 2000,100,100,2,2,2 };

        double phase4_B[6]{ 1000,1500,1500,50,50,50 };
        double phase4_M[6]{ 10,10,10,10,10,10 };

        double phase5_B[6]{ 100,100,100,10,10,10 };
        double phase5_M[6]{ 1,1,1,10,10,10 };


        //Counter
        int current_count = 0;
        int allign_count = 0;
        int success_count = 0;


        int start_count = 0;
        int complete_count = 0;
        int back_count = 0;

        //Pos Counter
        int pos_count = 0;
        int pos_success_count = 0;

        double current_pos_checkek[6] = {0};

        //Test
        //double actual_force[6]{ 0 };

        //Switch Angle
        double d_;

        //Switch Point
        int p_;

        //Arm1 Force Buffer
        std::array<double, 10> arm1_force_buffer[6] = {};
        int arm1_buffer_index[6]{ 0 };

        //Arm2 Force Buffer
        std::array<double, 10> arm2_force_buffer[6] = {};
        int arm2_buffer_index[6]{ 0 };

    };
    auto HoleInPeg::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        mout() << "Load P & L Vector" << std::endl;
//		gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
//		mout() << "Load Init Force" << std::endl;
//		imp_->init = true;
    }
    auto HoleInPeg::executeRT() -> int
    {
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

        //ver 1.0 not limit on vel, only limit force
        static double tolerance = 0.00005;
        //ver 2.0 limited vel, 20mm/s, 30deg/s
        static double max_vel[6]{ 0.00002,0.00002,0.00002,0.0005,0.0005,0.0005 };
        static double dead_zone[6]{ 0.1,0.1,0.1,0.0006,0.00920644,0.0006 };


        GravComp gc;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double arm1_comp_force[6]{ 0 };
        double arm1_current_pm[16]{ 0 };
        double arm1_current_pos[6]{ 0 };
        double arm1_actual_force[6]{ 0 };
        double arm1_filtered_force[6]{ 0 };
        double arm1_transform_force[6]{ 0 };

        double arm2_comp_force[6]{ 0 };
        double arm2_current_pm[16]{ 0 };
        double arm2_current_pos[6]{ 0 };
        double arm2_actual_force[6]{ 0 };
        double arm2_filtered_force[6]{ 0 };
        double arm2_transform_force[6]{ 0 };

        imp_->d_ = doubleParam("degree");

        imp_->p_ = int32Param("point");


        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };


        auto daJointMove = [&](double target_mp_[12])
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 12; i++)
            {
                if (current_angle[i] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] + move);
                }
                else if (current_angle[i] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i].setTargetPos(current_angle[i] - move);
                }
            }
        };

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        //single arm move 1-->white 2-->blue
        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };



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


        auto forceFilter = [&](double* actual_force_, double* filtered_force_, int m_)
        {
            if(m_ == 0)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm1_force_buffer[i][imp_->arm1_buffer_index[i]] = actual_force_[i];
                    imp_->arm1_buffer_index[i] = (imp_->arm1_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm1_force_buffer[i].begin(), imp_->arm1_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else if(m_ == 1)
            {
                for (int i = 0; i < 6; i++)
                {
                    imp_->arm2_force_buffer[i][imp_->arm2_buffer_index[i]] = actual_force_[i];
                    imp_->arm2_buffer_index[i] = (imp_->arm2_buffer_index[i] + 1) % 10;

                    filtered_force_[i] = std::accumulate(imp_->arm2_force_buffer[i].begin(), imp_->arm2_force_buffer[i].end(), 0.0) / 10;
                }
            }
            else
            {
                mout()<<"Wrong Filter!"<<std::endl;
            }

        };

        auto forceDeadZone = [&](double* actual_force_, double* area_)
        {
            for (int i = 0; i < 6; i++)
            {
                if (abs(actual_force_[i]) < area_[i])
                {
                    actual_force_[i] = 0;
                }
            }
        };

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        auto forceCheck = [&](double* current_force_, double* force_check_, int count_)
        {

            bool isValid = true;
            for (int i = 0; i < 3; i++)
            {
                if (abs(current_force_[i]) > force_check_[i])
                {
                    isValid = false;
                    break;
                }
            }

            if (isValid)
            {
                if (imp_->allign_count == 0) {
                    imp_->allign_count = count();
                }

                if ((count() - imp_->allign_count) % 30 == 0) {
                    imp_->success_count++;
                    mout() << "Check " << imp_->success_count << '\t' << "Current Count: " << count() << std::endl;
                    if (imp_->success_count >= count_)
                    {
                        return true;
                    }
                }
            }
            else
            {
                imp_->success_count = 0;
            }

            return false;
        };

        auto posCheck = [&](double* current_pos_, int count_)
        {
            if(count()%500 == 0)
            {
                std::copy(current_pos_, current_pos_+6, imp_->current_pos_checkek);
                //mout()<<"Save Current Pos: "<<imp_->current_pos_checkek[0]<<std::endl;
            }

            bool isValid = true;

            for(int i = 0; i < 3; i++)
            {
                if (abs(current_pos_[i] - imp_->current_pos_checkek[i]) >= 0.000005)
                {
                    isValid = false;
                    break;
                }
            }

            if(isValid)
            {
                if(imp_->pos_count == 0)
                {
                    imp_->pos_count = count();
                }

                if((count() - imp_->pos_count) % 50 == 0)
                {
                    imp_->pos_success_count ++;
                    //mout() << "Check " << imp_->pos_success_count << '\t' << "Current Count: " << count() << std::endl;
                    if(imp_->pos_success_count >= count_)
                    {
                        imp_->pos_count = 0;
                        imp_->pos_success_count = 0;
                        return true;
                    }
                }
            }
            else
            {
                imp_->pos_success_count = 0;
            }

            return false;
        };

        auto velDeadZone = [&](double vel_, double vel_limit_)
        {
            if (vel_ >= vel_limit_)
            {
                vel_ = vel_limit_;
            }
            else if (vel_ <= -vel_limit_)
            {
                vel_ = -vel_limit_;
            }

        };



        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);




        if(!imp_->init)
        {
            double assem_pos[6]{ -0.726, 0.009930, 0.289672, PI / 4, -PI / 2, - PI / 4 };
            double init_angle[6]{0};

            model_a2.setOutputPos(assem_pos);
            if(model_a2.inverseKinematics())
            {
                mout()<<"Error"<<std::endl;
            }

            model_a2.getInputPos(init_angle);

            if(count()%1000 == 0)
            {
                mout()<<"Init angle: "<<init_angle[0]<<'\t'<<init_angle[1]<<'\t'<<init_angle[2]<<'\t'
                        <<init_angle[3]<<'\t'<<init_angle[4]<<'\t'<<init_angle[5]<<std::endl;
                mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                        <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
            }

            saJointMove(init_angle, 1);

            if(motorsPositionCheck(current_sa_angle, init_angle, 6))
            {
                getForceData(imp_->arm1_init_force, 0, imp_->init);
                getForceData(imp_->arm2_init_force, 1, imp_->init);

                gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

                mout()<<"Init Complete"<<std::endl;

                imp_->init = true;
            }
        }
        else
        {

            eeA1.getP(arm1_current_pos);
            eeA1.getMpm(arm1_current_pm);

            eeA2.getP(arm2_current_pos);
            eeA2.getMpm(arm2_current_pm);

            //Force Comp, Filtered, Transform
            getForceData(arm1_actual_force, 0, imp_->init);
            gc.getCompFT(arm1_current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, arm1_comp_force);

            getForceData(arm2_actual_force, 1, imp_->init);
            gc.getCompFT(arm2_current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, arm2_comp_force);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_comp_force[i] = arm1_actual_force[i] + arm1_comp_force[i];
                arm2_comp_force[i] = arm2_actual_force[i] + arm2_comp_force[i];
            }

            forceFilter(arm1_comp_force, arm1_filtered_force, 0);
            forceFilter(arm2_comp_force, arm2_filtered_force, 1);

            //forceDeadZone(arm1_filtered_force, arm1_dead_zone);
            //forceDeadZone(arm2_filtered_force, arm2_dead_zone);

            forceTransform(arm1_filtered_force, arm1_transform_force, 0);
            forceTransform(arm2_filtered_force, arm2_transform_force, 1);

            for (size_t i = 0; i < 6; i++)
            {
                arm1_transform_force[i] -= imp_->arm1_start_force[i];
                arm2_transform_force[i] -= imp_->arm2_start_force[i];
            }


            //Phase 1 Approach to The Hole
            if (!imp_->phase1)
            {
                //Tool
                double assem_pos[6]{ -0.726, 0.009930, 0.289672, PI / 4, -PI / 2, - PI / 4 };
                double assem_angle[6]{ 0 };
                double assem_rm[9]{ 0 };

                //Define Initial Rotate displacment
                double rotate_angle[3]{ 0, 0, imp_->d_ * 2 * PI / 360 };
                double rotate_rm[9]{ 0 };
                double desired_rm[9]{ 0 };

                aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
                aris::dynamic::s_re2rm(assem_pos + 3, assem_rm, "321");
                aris::dynamic::s_mm(3, 3, 3, rotate_rm, assem_rm, desired_rm);
                aris::dynamic::s_rm2re(desired_rm, assem_pos + 3, "321");


                //Define Initial Position displacment
                std::array<std::array<double, 3>, 13> points =
                {
                    {
                        {0, -0.007, 0},
                        {0, -0.009, 0},
                        {0, -0.011, 0},
                        {0, -0.013, 0},
                        {0, -0.015, 0},
                        {0, -0.017, 0},
                        {0, -0.020, 0},
                        {0, 0.007, 0},
                        {0, 0.009, 0},
                        {0, 0.011, 0},
                        {0, 0.013, 0},
                        {0, 0.015, 0},
                        {0, 0.017, 0},
                    }
                };

                if(imp_->p_ < 0 || imp_->p_ >= points.size())
                {
                    mout()<<"Error Points Index"<<std::endl;
                    return 0;
                }

//                for(size_t i = 0; i < 3; i++)
//                {
//                    assem_pos[i] += points[imp_->p_][i];
//                }

                //Compensate pos error due to rotate
                //x
                assem_pos[0] += points[imp_->p_][0];
                //y

                if(rotate_angle[2] < 0)
                {
                    assem_pos[1] += (-0.003 + points[imp_->p_][1] / std::cos(abs(rotate_angle[2])));
                }
                else if(rotate_angle[2] > 0)
                {
                    assem_pos[1] += (0.003 + points[imp_->p_][1] / std::cos(abs(rotate_angle[2])));
                }
                else
                {
                     assem_pos[1] += points[imp_->p_][1];
                }


                //z
                assem_pos[2] += (points[imp_->p_][2] / std::cos(abs(rotate_angle[1])));






                eeA2.setP(assem_pos);


                if (model_a2.inverseKinematics())
                {
                    mout() << "Assem Pos Inverse Failed" << std::endl;
                }

                model_a2.getInputPos(assem_angle);

                saJointMove(assem_angle, 1);
                if (motorsPositionCheck(current_sa_angle, assem_angle, 6))
                {

                    imp_->phase1 = true;
                    imp_->start_count = count();

                    mout()<<"Current Angle: "<< rotate_angle[0] <<'\t'<< rotate_angle[1] <<'\t' << rotate_angle[2]<<std::endl;
                    mout()<<"Current Pos: "<< assem_pos[0] <<'\t'<< assem_pos[1] <<'\t' << assem_pos[2]<<std::endl;
                    mout() << "Assembly Start !" << std::endl;
                }

            }
            //Phase 2 Contact, Have Certain Position Adjustment
            else if (imp_->phase1 && !imp_->phase2)
            {

                if(count() == imp_->start_count + 2000)
                {


                    for (size_t i = 0; i < 6; i++)
                    {
                        imp_->arm1_start_force[i] = arm1_transform_force[i];
                        imp_->arm2_start_force[i] = arm2_transform_force[i];
                    }
                    mout()<<"Start Force Comp"<<std::endl;
                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;


                }


                double raw_force_checker[6]{ 0 };
                double comp_force_checker[6]{ 0 };
                double force_checker[6]{ 0 };

                double a2_pm[16]{ 0 };
                eeA2.getMpm(a2_pm);
                eeA2.getP(arm2_current_pos);

                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                        << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;

//                    mout()<<"A2 Start Force: "<<imp_->arm2_start_force[0]<<'\t'<<imp_->arm2_start_force[1]<<'\t'<<imp_->arm2_start_force[2]<<'\t'
//                            <<imp_->arm2_start_force[3]<<'\t'<<imp_->arm2_start_force[4]<<'\t'<<imp_->arm2_start_force[5]<<std::endl;

                }


                //Arm1
                getForceData(raw_force_checker, 1, imp_->init);
                gc.getCompFT(a2_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force_checker);
                if (count() % 1000 == 0)
                {
                    mout() << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                }

                for (int i = 0; i < 6; i++)
                {
                    force_checker[i] = comp_force_checker[i] + raw_force_checker[i];
                    if (abs(force_checker[i]) >= 0.5)
                    {
                        imp_->phase2 = true;
                        mout() << "Contact Check" << std::endl;
                        mout() << "Contact Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                            << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                        mout() << "Contact force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                            << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                        break;


                    }

                }
                if (!imp_->phase2)
                {
                    arm2_current_pos[0] -= 0.000003;
                    saMove(arm2_current_pos, model_a2, 1);
                }

            }
            //Phase 3 Pose Adjust, Position Decised, Pose Move Only
            else if (imp_->phase2 && !imp_->phase3)
            {

                double acc[3]{ 0 };
                double ome[3]{ 0 };

                double dx[3]{ 0 };
                double dth[3]{ 0 };
                double dt = 0.001;

                double desired_force[6]{ 0,0,0,0,0,0 };

                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_transform_force[i]) > 15.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                            << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout() << "force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                           << "pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << std::endl;
                }

                if (posCheck(arm2_current_pos, 8))
                {
                    imp_->phase3 = true;
                    mout() << "Pos Complete" << std::endl;
                    mout() << "Complete Pos: " << arm2_current_pos[0] << '\t' << arm2_current_pos[1] << '\t' << arm2_current_pos[2] << '\t'
                        << arm2_current_pos[3] << '\t' << arm2_current_pos[4] << '\t' << arm2_current_pos[5] << std::endl;
                    mout() << "Complete force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                        << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;

                    imp_->complete_count = count();

                }
                else
                {

                    if(arm2_transform_force[0] >= (imp_->phase3_fd[0] - 0.01) && arm2_transform_force[0] <= (imp_->phase3_fd[0] + 2))
                    {
                        arm2_transform_force[0] = imp_->phase3_fd[0];
                    }


                    //Impedence Controller
                    for (int i = 0; i < 1; i++)
                    {
                        // da = (Fd-Fe-Bd*(v-vd)-k*(x-xd))/M
                        acc[i] = (-imp_->phase3_fd[i] + arm2_transform_force[i] - imp_->phase3_B[i] * (imp_->v_c[i] - imp_->v_d[i])) / imp_->phase3_M[i];
                    }


                    for (int i = 0; i < 1; i++)
                    {
                        imp_->v_c[i] += acc[i] * dt;
                        velDeadZone(imp_->v_c[i], max_vel[i]);
                        dx[i] = imp_->v_c[i] * dt + acc[i] * dt * dt;
                        arm2_current_pos[i] = dx[i] + arm2_current_pos[i];

                    }


                    saMove(arm2_current_pos, model_a2, 1);
                }
            }
            //Phase 4 Get Data
            else if (imp_->phase3 && !imp_->phase4)
            {
                if(count() <= imp_->complete_count + 1550)
                {
                    if(count() % 50 == 0)
                    {
                        mout()<<"A1_Force"<<"\t"<<arm1_transform_force[0]<<"\t"<<arm1_transform_force[1]<<"\t"<<arm1_transform_force[2]<<"\t"
                                <<arm1_transform_force[3]<<"\t"<<arm1_transform_force[4]<<"\t"<<arm1_transform_force[5]<<"\t"<<"\t"<<"\t"<<"\t"
                               <<"A2_Force"<<"\t"<<arm2_transform_force[0]<<"\t"<<arm2_transform_force[1]<<"\t"<<arm2_transform_force[2]<<"\t"
                               <<arm2_transform_force[3]<<"\t"<<arm2_transform_force[4]<<"\t"<<arm2_transform_force[5]<<std::endl;
                    }
                }
                else
                {
                    mout()<<"Data Acquired! Current Point: "<< imp_->p_ << '\t' <<"Current Angle: "<< imp_->d_ <<std::endl;
                    imp_->back_count = count();
                    imp_->phase4 = true;
                }

            }
            //Back
            else if(imp_->phase4 && !imp_->phase5)
            {
                //Safety Check
                for (size_t i = 0; i < 3; i++)
                {
                    if (abs(arm2_transform_force[i]) > 15.0)
                    {
                        mout() << "Emergency Brake" << std::endl;
                        mout() << "Brake force: " << arm2_transform_force[0] << '\t' << arm2_transform_force[1] << '\t' << arm2_transform_force[2] << '\t'
                            << arm2_transform_force[3] << '\t' << arm2_transform_force[4] << '\t' << arm2_transform_force[5] << std::endl;
                        return 0;
                    }
                }
                if (count() % 100 == 0)
                {
                    mout()<<"Complete!: "<<"A1_Force"<<"\t"<<arm1_transform_force[0]<<"\t"<<arm1_transform_force[1]<<"\t"<<arm1_transform_force[2]<<"\t"
                            <<arm1_transform_force[3]<<"\t"<<arm1_transform_force[4]<<"\t"<<arm1_transform_force[5]<<"\t"<<"\t"
                           <<"A2_Force"<<"\t"<<arm2_transform_force[0]<<"\t"<<arm2_transform_force[1]<<"\t"<<arm2_transform_force[2]<<"\t"
                           <<arm2_transform_force[3]<<"\t"<<arm2_transform_force[4]<<"\t"<<arm2_transform_force[5]<<std::endl;
                }


                if(count() <= imp_->back_count + 4000)
                {
                    arm2_current_pos[0] += 0.00001;
                    saMove(arm2_current_pos, model_a2, 1);
                }
                else
                {
                    mout()<<"Test Complete! "<< "Points: " << imp_->p_ << '\t' << "Angle: " << imp_->d_ <<std::endl;
                    imp_->phase5 = true;
                    return 0;
                }
            }

        }


        return 40000 - count();
    }
    HoleInPeg::HoleInPeg(const std::string& name)
{
        aris::core::fromXmlString(command(),
         "<Command name=\"m_hp\">"
         "	<GroupParam>"
         "	<Param name=\"degree\" default=\"0\" abbreviation=\"d\"/>"
         "	<Param name=\"point\" default=\"0\" abbreviation=\"p\"/>"
         "	</GroupParam>"
         "</Command>");
}
    HoleInPeg::~HoleInPeg() = default;


	struct PegXYZ::Imp {

		//Flag
		bool init = false;

		//Arm1
		double arm1_init_force[6]{ 0 };
		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };

		//Arm2
		double arm2_init_force[6]{ 0 };
		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };
		
		//Switch Model
		int m_;
		//Switch Direction
		int d_;
		//Switch Amplitude
		double a_;

		//Force Buffer
		std::array<double, 10> force_buffer[6] = {};
		int buffer_index[6]{ 0 };

		};
	auto PegXYZ::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		// GravComp gc;
		// gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		// mout() << "Load P & L Vector" << std::endl;
		// gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
		// mout() << "Load Init Force" << std::endl;
	}
	auto PegXYZ::executeRT() -> int
	{
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


		static double d_pos = 0.000005;

		GravComp gc;

		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };

		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double filtered_force[6]{ 0 };
		double transform_force[6]{ 0 };

		imp_->m_ = int32Param("model");
		imp_->d_ = int32Param("direction");
		imp_->a_ = doubleParam("amplitude");


		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}

			}

		};

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

		auto forceDeadZone = [&](double* actual_force_,  double* area_)
		{
			for (int i = 0; i < 6; i++)
			{
				if(abs(actual_force_[i]) < area_[i])
				{
					actual_force_[i] = 0;
				}
			}
		};

		auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
		{
			if(m_ == 0)
			{
				transform_force_[0] = filtered_force[2];
				transform_force_[1] = -filtered_force[1];
				transform_force_[2] = filtered_force[0];

				transform_force_[3] = filtered_force[5];
				transform_force_[4] = -filtered_force[4];
				transform_force_[5] = filtered_force[3];
			}
			else if(m_ == 1)
			{
				transform_force_[0] = -filtered_force[2];
				transform_force_[1] = filtered_force[1];
				transform_force_[2] = filtered_force[0];

				transform_force_[3] = -filtered_force[5];
				transform_force_[4] = filtered_force[4];
				transform_force_[5] = filtered_force[3];
			}
			else
			{
				mout()<<"Error Model In Force Transform"<<std::endl;
			}
		};

		auto forceCheck = [&](double* current_force_, double* force_check_)
		{
			for (int i = 0; i < 6; i++)
			{
				if (abs(current_force_[i]) > force_check_[i])
				{
					return false;
				}
			}

			return true;
		};

		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
			}
		};

		// getForceData(actual_force, 0, true);
		// gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
		// for (size_t i = 0; i < 6; i++)
		// {
		// 	comp_force[i] = actual_force[i] + comp_force[i];
		// }
		//forceFilter(comp_force, filtered_force);

		//forceTransform(filtered_force, transform_force, 0);
	
		//Safety Check
		for (size_t i = 0; i < 3; i++)
		{
			if (abs(transform_force[i]) > 15.0)
			{
				mout()<<"Emergency Brake"<<std::endl;
				return 0;
			}
		}

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}

		std::copy(current_angle, current_angle + 6, current_sa_angle);
		if(count() == 1)
		{
			dualArm.setInputPos(current_angle);
			if(dualArm.forwardKinematics())
			{
				mout()<<"Init Forward Failed"<<std::endl;
			}
		}

		eeA1.getP(current_pos);

		// if (count() % 1000 == 0) 
		// {
		// 	mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
		// 	<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
		// }

		// if (count() % 10 == 0) 
		// {
		// 	mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
		// 	<< transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;
		// }
			
		if(imp_->d_ == 0 || imp_->d_ == 1 || imp_->d_ == 2)
		{
			current_pos[imp_->d_] += d_pos*imp_->a_;
		}
		else
		{
			mout()<<"Wrong Direction"<<std::endl;
			return 0;
		}
		
		saMove(current_pos, model_a1, imp_->m_);

		if(count() == 100)
		{
			mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
			<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
		}

		return 100 - count();
	}
	PegXYZ::PegXYZ(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"x\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	<Param name=\"direction\" default=\"0\" abbreviation=\"d\"/>"
			"	<Param name=\"amplitude\" default=\"1\" abbreviation=\"a\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	PegXYZ::~PegXYZ() = default;

	//Not Finished
	struct PegRPY::Imp {

		//Flag
		bool init = false;

		//Arm1
		double arm1_init_force[6]{ 0 };
		double arm1_p_vector[6]{ 0 };
		double arm1_l_vector[6]{ 0 };

		//Arm2
		double arm2_init_force[6]{ 0 };
		double arm2_p_vector[6]{ 0 };
		double arm2_l_vector[6]{ 0 };

		//Switch Model
		int m_;
		//Switch Direction
		int d_;
		//Switch Amplitude
		double a_;

		//Force Buffer
		std::array<double, 10> force_buffer[6] = {};
		int buffer_index[6]{ 0 };


		};
	auto PegRPY::prepareNrt() -> void
	{
		for (auto& m : motorOptions()) m =
			aris::plan::Plan::CHECK_NONE |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		// GravComp gc;
		// gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
		// mout() << "Load P & L Vector" << std::endl;
		// gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
		// mout() << "Load Init Force" << std::endl;
	}
	auto PegRPY::executeRT() -> int
	{
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

		GravComp gc;

		double current_angle[12]{ 0 };
		double current_sa_angle[6]{ 0 };

		double comp_force[6]{ 0 };
		double current_pm[16]{ 0 };
		double current_pos[6]{ 0 };
		double current_force[6]{ 0 };
		double actual_force[6]{ 0 };
		double filtered_force[6]{ 0 };
		double transform_force[6]{ 0 };

		imp_->m_ = int32Param("model");
		imp_->d_ = int32Param("direction");
		imp_->a_ = doubleParam("amplitude");


		auto getForceData = [&](double* data_, int m_, bool init_)
		{

			int raw_force[6]{ 0 };

			for (std::size_t i = 0; i < 6; ++i)
			{
				if (ecMaster()->slavePool()[9 + 9 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
					mout() << "error" << std::endl;

				data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

			}

			if (!init_)
			{
				mout() << "Compensate Init Force" << std::endl;
			}
			else
			{
				if (m_ == 0)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

					}
				}
				else if (m_ == 1)
				{
					for (std::size_t i = 0; i < 6; ++i)
					{

						data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

					}
				}
				else
				{
					mout() << "Wrong Model" << std::endl;
				}

			}

		};

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

		auto forceDeadZone = [&](double* actual_force_,  double* area_)
		{
			for (int i = 0; i < 6; i++)
			{
				if(abs(actual_force_[i]) < area_[i])
				{
					actual_force_[i] = 0;
				}
			}
		};

		auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
		{
			if(m_ == 0)
			{
				transform_force_[0] = filtered_force[2];
				transform_force_[1] = -filtered_force[1];
				transform_force_[2] = filtered_force[0];

				transform_force_[3] = filtered_force[5];
				transform_force_[4] = -filtered_force[4];
				transform_force_[5] = filtered_force[3];
			}
			else if(m_ == 1)
			{
				transform_force_[0] = -filtered_force[2];
				transform_force_[1] = filtered_force[1];
				transform_force_[2] = filtered_force[0];

				transform_force_[3] = -filtered_force[5];
				transform_force_[4] = filtered_force[4];
				transform_force_[5] = filtered_force[3];
			}
			else
			{
				mout()<<"Error Model In Force Transform"<<std::endl;
			}
		};

		auto forceCheck = [&](double* current_force_, double* force_check_)
		{
			for (int i = 0; i < 6; i++)
			{
				if (abs(current_force_[i]) > force_check_[i])
				{
					return false;
				}
			}

			return true;
		};

		auto forceFilter = [&](double* actual_force_, double* filtered_force_)
		{
			for (int i = 0; i < 6; i++)
			{
				imp_->force_buffer[i][imp_->buffer_index[i]] = actual_force_[i];
				imp_->buffer_index[i] = (imp_->buffer_index[i] + 1) % 10;

				filtered_force_[i] = std::accumulate(imp_->force_buffer[i].begin(), imp_->force_buffer[i].end(), 0.0) / 10;
			}
		};

		// getForceData(actual_force, 0, true);
		// gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
		// for (size_t i = 0; i < 6; i++)
		// {
		// 	comp_force[i] = actual_force[i] + comp_force[i];
		// }
		//forceFilter(comp_force, filtered_force);

		//forceTransform(filtered_force, transform_force, 0);
	
		//Safety Check
		for (size_t i = 0; i < 3; i++)
		{
			if (abs(transform_force[i]) > 15.0)
			{
				mout()<<"Emergency Brake"<<std::endl;
				return 0;
			}
		}

		for (int i = 0; i < 12; i++)
		{
			current_angle[i] = controller()->motorPool()[i].actualPos();
		}

		std::copy(current_angle, current_angle + 6, current_sa_angle);


		if(count() == 1)
		{
			dualArm.setInputPos(current_angle);
			if(dualArm.forwardKinematics())
			{
				mout()<<"Init Forward Failed"<<std::endl;
			}
		}

		eeA1.getP(current_pos);

		// if (count() % 1000 == 0) 
		// {
		// 	mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
		// 	<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
		// }

		// if (count() % 10 == 0) 
		// {
		// 	mout() << "force: " << transform_force[0] << '\t' << transform_force[1] << '\t' << transform_force[2] << '\t'
		// 	<< transform_force[3] << '\t' << transform_force[4] << '\t' << transform_force[5] << std::endl;
		// }


		//Define Initial Rotate Error
		double rotate_angle[3]{ 0 };
		double rotate_rm[9]{ 0 };
		double desired_rm[9]{ 0 };
		double current_rm[6]{0};

		if(!imp_->d_ == 0 && !imp_->d_ == 1 && !imp_->d_ == 2)
		{
			mout()<<"Wrong Direction"<<std::endl;
			return 0;
		}

		rotate_angle[imp_->d_] = 0.01 * imp_->a_;

		aris::dynamic::s_ra2rm(rotate_angle, rotate_rm);
		aris::dynamic::s_re2rm(current_pos + 3, current_rm, "321");
		aris::dynamic::s_mm(3, 3, 3, rotate_rm, current_rm, desired_rm);
		aris::dynamic::s_rm2re(desired_rm, current_pos + 3, "321");
			
		

		saMove(current_pos, model_a1, imp_->m_);

		if(count() == 5000)
		{
			mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
			<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
		}

		return 5000 - count();
	}
	PegRPY::PegRPY(const std::string& name)
	{
		aris::core::fromXmlString(command(),
			"<Command name=\"r\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
			"	<Param name=\"amplitude\" default=\"1\" abbreviation=\"a\"/>"
			"	</GroupParam>"
			"</Command>");
	}
	PegRPY::~PegRPY() = default;
	
    struct PegOutHole::Imp {

        //Flag
        bool init = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

		//Switch Model
		int m_;

    };
    auto PegOutHole::prepareNrt() -> void
    {
        for (auto& m : motorOptions()) m =
            aris::plan::Plan::CHECK_NONE |
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

        GravComp gc;
        // gc.loadPLVector(imp_->arm1_p_vector, imp_->arm1_l_vector, imp_->arm2_p_vector, imp_->arm2_l_vector);
        // mout() << "Load P & L Vector" << std::endl;
        // gc.loadInitForce(imp_->arm1_init_force, imp_->arm2_init_force);
        // mout()<<"Load Init Force"<<std::endl;
    }
    auto PegOutHole::executeRT() -> int
    {
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

		imp_->m_ = int32Param("model");


        GravComp gc;

        static double d_pos = 0.00001;

        double current_angle[12]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double actual_force[6]{ 0 };
        double transform_force[6]{ 0 };


        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

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

        auto forceTransform = [&](double* actual_force_, double* transform_force_, int m_)
        {
            if (m_ == 0)
            {
                transform_force_[0] = actual_force_[2];
                transform_force_[1] = -actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = actual_force_[5];
                transform_force_[4] = -actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else if (m_ == 1)
            {
                transform_force_[0] = -actual_force_[2];
                transform_force_[1] = actual_force_[1];
                transform_force_[2] = actual_force_[0];

                transform_force_[3] = -actual_force_[5];
                transform_force_[4] = actual_force_[4];
                transform_force_[5] = actual_force_[3];
            }
            else
            {
                mout() << "Error Model In Force Transform" << std::endl;
            }
        };

        for(int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }

        if(count() == 1)
        {
            dualArm.setInputPos(current_angle);
            if(dualArm.forwardKinematics()){mout()<<"Error"<<std::endl;}
            mout()<<"Init"<<std::endl;
        }


		if(imp_->m_ == 0)
		{

			eeA1.getP(current_pos);
			eeA1.getMpm(current_pm);

			// getForceData(actual_force, 0, true);
			// gc.getCompFT(current_pm, imp_->arm1_l_vector, imp_->arm1_p_vector, comp_force);
			// for (size_t i = 0; i < 6; i++)
			// {
			// 	comp_force[i] = actual_force[i] + comp_force[i];
			// }

			// forceTransform(comp_force, transform_force, 0);

			//Safety Check
			for (size_t i = 0; i < 3; i++)
			{
				if (abs(transform_force[i]) > 30)
				{
					mout() << "Emergency Brake" << std::endl;
					return 0;
				}
			}

			if (count() % 1000 == 0)
			{
				mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
					<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
			}


			current_pos[0] -= d_pos;

			saMove(current_pos, model_a1, 0);

		}
		else if(imp_->m_ == 1)
		{

			eeA2.getP(current_pos);
			eeA2.getMpm(current_pm);

			// getForceData(actual_force, 1, true);
			// gc.getCompFT(current_pm, imp_->arm2_l_vector, imp_->arm2_p_vector, comp_force);
			// for (size_t i = 0; i < 6; i++)
			// {
			// 	comp_force[i] = actual_force[i] + comp_force[i];
			// }

			// forceTransform(comp_force, transform_force, 1);

			//Safety Check
			for (size_t i = 0; i < 3; i++)
			{
				if (abs(transform_force[i]) > 30)
				{
					mout() << "Emergency Brake" << std::endl;
					return 0;
				}
			}

			if (count() % 1000 == 0)
			{
				mout() << "pos: " << current_pos[0] << '\t' << current_pos[1] << '\t' << current_pos[2] << '\t'
					<< current_pos[3] << '\t' << current_pos[4] << '\t' << current_pos[5] << std::endl;
			}


			current_pos[0] += d_pos;

			saMove(current_pos, model_a2, 1);
		}
		else
		{
			mout()<<"Wrong Model"<<std::endl;
			return 0;
		}

        return 8000 - count();
    }
    PegOutHole::PegOutHole(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"m_po\">"
			"	<GroupParam>"
			"	<Param name=\"model\" default=\"0\" abbreviation=\"m\"/>"
			"	</GroupParam>"
			"</Command>");
    }
    PegOutHole::~PegOutHole() = default;


   	struct Arm2Init::Imp {

        //Flag
        bool init = false;

        //Arm1
        double arm1_init_force[6]{ 0 };
        double arm1_p_vector[6]{ 0 };
        double arm1_l_vector[6]{ 0 };

        //Arm2
        double arm2_init_force[6]{ 0 };
        double arm2_p_vector[6]{ 0 };
        double arm2_l_vector[6]{ 0 };

    };
    auto Arm2Init::prepareNrt()->void {

        for (auto& m : motorOptions()) m =
            aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;


    }
    auto Arm2Init::executeRT()->int {

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


        GravComp gc;

        static double d_pos = 0.00001;
        static double tolerance = 0.00005;

        double current_angle[12]{ 0 };
        double current_sa_angle[6]{ 0 };

        double comp_force[6]{ 0 };
        double current_pm[16]{ 0 };
        double current_pos[6]{ 0 };
        double actual_force[6]{ 0 };
        double transform_force[6]{ 0 };



        auto getForceData = [&](double* data_, int m_, bool init_)
        {

            int raw_force[6]{ 0 };

            for (std::size_t i = 0; i < 6; ++i)
            {
                if (ecMaster()->slavePool()[8 + 7 * m_].readPdo(0x6020, 0x01 + i, raw_force + i, 32))
                    mout() << "error" << std::endl;

                data_[i] = (static_cast<double>(raw_force[i]) / 1000.0);

            }

            if (!init_)
            {
                mout() << "Compensate Init Force" << std::endl;
            }
            else
            {
                if (m_ == 0)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm1_init_force[i];

                    }
                }
                else if (m_ == 1)
                {
                    for (std::size_t i = 0; i < 6; ++i)
                    {

                        data_[i] = (static_cast<double>(raw_force[i]) / 1000.0) - imp_->arm2_init_force[i];

                    }
                }
                else
                {
                    mout() << "Wrong Model" << std::endl;
                }

            }

        };

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

        auto motorsPositionCheck = [](const double* current_sa_angle_, const double* target_pos_, size_t dim_)
        {
            for (int i = 0; i < dim_; i++)
            {
                if (std::fabs(current_sa_angle_[i] - target_pos_[i]) >= tolerance)
                {
                    return false;
                }
            }

            return true;
        };

        //single arm move 1-->white 2-->blue
        auto saJointMove = [&](double target_mp_[6], int m_)
        {
            double current_angle[12] = { 0 };
            double move = 0.00005;

            for (int i = 0; i < 12; i++)
            {
                current_angle[i] = controller()->motorPool()[i].targetPos();
            }

            for (int i = 0; i < 6; i++)
            {
                if (current_angle[i+6*m_] <= target_mp_[i] - move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] + move);
                }
                else if (current_angle[i+6*m_] >= target_mp_[i] + move)
                {
                    controller()->motorPool()[i+6*m_].setTargetPos(current_angle[i+6*m_] - move);
                }
            }
        };


        for (int i = 0; i < 12; i++)
        {
            current_angle[i] = controller()->motorPool()[i].actualPos();
        }


        std::copy(current_angle + 6, current_angle + 12, current_sa_angle);





        double assem_pos[6]{ -0.726, 0.009930, 0.289672, PI / 4, -PI / 2, - PI / 4 };
        double init_angle[6]{0};

        model_a2.setOutputPos(assem_pos);
        if(model_a2.inverseKinematics())
        {
            mout()<<"Error"<<std::endl;
        }

        model_a2.getInputPos(init_angle);

        if(count()%1000 == 0)
        {
            mout()<<"current angle: "<<current_sa_angle[0]<<'\t'<<current_sa_angle[1]<<'\t'<<current_sa_angle[2]<<'\t'
                    <<current_sa_angle[3]<<'\t'<<current_sa_angle[4]<<'\t'<<current_sa_angle[5]<<std::endl;
        }

        saJointMove(init_angle, 1);

        if(motorsPositionCheck(current_sa_angle, init_angle, 6))
        {
            getForceData(imp_->arm1_init_force, 0, imp_->init);
            getForceData(imp_->arm2_init_force, 1, imp_->init);

            gc.saveInitForce(imp_->arm1_init_force, imp_->arm2_init_force);

            mout()<<"Init Complete"<<std::endl;

            imp_->init = true;
            return 0;
        }



        if(count() == 20000)
        {
            mout()<<"Over Time"<<std::endl;
        }

        return 20000 - count();




    }
    Arm2Init::Arm2Init(const std::string& name)
    {
        aris::core::fromXmlString(command(),
            "<Command name=\"2_back\"/>");
    }
    Arm2Init::~Arm2Init() = default;




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
		aris::core::class_<ForceAlign>("ForceAlign")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ForceKeep>("ForceKeep")
			.inherit<aris::plan::Plan>();
		aris::core::class_<ForceDrag>("ForceDrag")
			.inherit<aris::plan::Plan>();
		aris::core::class_<PegInHole>("PegInHole")
			.inherit<aris::plan::Plan>();
		aris::core::class_<PegOutHole>("PegOutHole")
			.inherit<aris::plan::Plan>();
		aris::core::class_<PegXYZ>("PegXYZ")
			.inherit<aris::plan::Plan>();
		aris::core::class_<PegRPY>("PegRPY")
			.inherit<aris::plan::Plan>();
		aris::core::class_<HoleInPeg>("HoleInPeg")
			.inherit<aris::plan::Plan>();
		aris::core::class_<Arm2Init>("Arm2Init")
			.inherit<aris::plan::Plan>();

	}





}





