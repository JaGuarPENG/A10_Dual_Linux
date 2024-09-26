#include"plan.hpp"
#include<cmath>
#include<iostream>
#include <aris.hpp>


using namespace std;



auto cosCurve::getCurve(int count)->double
{
    int t = count + 1;
    double s = 0;
    s = a_ * std::cos(w_ * t / 1000.0 + p_);

    return s;
}





auto TCurve::getTCurve(int count)->double
{
	//double ta = p.ta_;
	//double a = p.a_;
	//double v = p.v_;
	//double T_c = p.Tc_;
	int t = count + 1;
	double s = 0;

	if (2 * ta_ == Tc_)   //三角形曲线  正弦波
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	else    //梯形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	//std::cout << s << std::endl;
	return s;
}

//计算梯形曲线的参数，由成员函数初始化，对应输入参数由构造函数初始化
auto TCurve::getCurveParam()->void
{
	if (v_ * v_ / a_ <= 1)
	{
		this->Tc_ = (a_ + v_ * v_) / v_ / a_;
		this->a_ = a_;
		this->v_ = v_;
	}
	else
	{
		//安速度计算，此时给定的加速度不起作用
		this->Tc_ = 2.0 / v_;
		this->a_ = v_ * v_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}



auto TCurve2::getCurveParam()->void
{
	this->ta_ = v_ / a_;
	this->pa_ = 0.5 * a_ * ta_ * ta_;
	this->tm_ = (p_ - 2 * pa_) / v_;
	this->tc_ = tm_ + 2 * ta_;
}



auto TCurve2::getTCurve(int count)->double
{
	int t = count + 1;
	double s = 0;

	if (t < tc_ * 1000 + 1)
	{
		if (tc_ - 2 * ta_ > 0)
		{
			if (t < ta_ * 1000 + 1)
			{
				s = 0.5 * a_ * (t / 1000.0) * (t / 1000.0);
			}
			else if (ta_ * 1000 < t && t < tc_ * 1000 - ta_ * 1000 + 1)
			{
				s = 0.5 * a_ * ta_ * ta_ + a_ * ta_ * (t / 1000.0 - ta_);
			}
			else
			{
				s = p_ - 0.5 * a_ * (tc_ - t / 1000.0) * (tc_ - t / 1000.0);
			}
		}

		else
		{
			ta_ = sqrt(p_ / a_);
			tc_ = 2 * ta_;

			if (t < ta_ * 1000 + 1)
			{
				s = 0.5 * a_ * (t / 1000.0) * (t / 1000.0);
			}
			else
			{
				s = p_ - 0.5 * a_ * (tc_ - t / 1000.0) * (tc_ - t / 1000.0);
			}
		}


		return s;
	}
	else
	{
		return p_;
	}

}


//EE Force Compensate Caculation
//Refer to 10.16383/j.aas.2017.c150753

//p = [x, y, z, k1, k2, k3]T
//m = [mx1, my1, mz1, ... mx3, my3, mz3]T
//p = inv(F.t * F) * F.t * m


//l = [Lx, Ly, Lz, Fx0, Fy0, Fz0]T
//f = [Fx1, Fy1, Fz1, ... Fx3, Fy3, Fz3]T
//l = inv(R.t * R) * R.t * f

// 实现 getTorqueVector 方法
auto GravComp::getTorqueVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double torque_vector_[9]) -> void {
	std::copy(force_data1_ + 3, force_data1_ + 6, torque_vector_);
	std::copy(force_data2_ + 3, force_data2_ + 6, torque_vector_ + 3);
	std::copy(force_data3_ + 3, force_data3_ + 6, torque_vector_ + 6);
}

// 实现 getForceVector 方法
auto GravComp::getForceVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double force_vector_[9]) -> void {
	std::copy(force_data1_, force_data1_ + 3, force_vector_);
	std::copy(force_data2_, force_data2_ + 3, force_vector_ + 3);
	std::copy(force_data3_, force_data3_ + 3, force_vector_ + 6);
}

// 实现 getTempFMatrix 方法
auto GravComp::getTempFMatrix(double force_data_[6], double temp_[18]) -> void {
	double tempInit[18] = { 0, force_data_[2], -force_data_[1], 1, 0, 0,
							-force_data_[2], 0, force_data_[0], 0, 1, 0,
							force_data_[1], -force_data_[0], 0, 0, 0, 1 };
	std::copy(tempInit, tempInit + 18, temp_);
}

// 实现 getTempRMatrix 方法
auto GravComp::getTempRMatrix(double pose_matrix_[9], double temp_[18]) -> void {
	double tempInit[18] = { pose_matrix_[0], pose_matrix_[3], pose_matrix_[6], 1, 0, 0,
							pose_matrix_[1], pose_matrix_[4], pose_matrix_[7], 0, 1, 0,
							pose_matrix_[2], pose_matrix_[5], pose_matrix_[8], 0, 0, 1 };
	std::copy(tempInit, tempInit + 18, temp_);
}

// 实现 getFMatrix 方法
auto GravComp::getFMatrix(double force_data1_[6], double force_data2_[6], double force_data3_[6], double f_matrix_[54]) -> void {
	double temp1[18] = { 0 };
	double temp2[18] = { 0 };
	double temp3[18] = { 0 };

	getTempFMatrix(force_data1_, temp1);
	getTempFMatrix(force_data2_, temp2);
	getTempFMatrix(force_data3_, temp3);

	std::copy(temp1, temp1 + 18, f_matrix_);
	std::copy(temp2, temp2 + 18, f_matrix_ + 18);
	std::copy(temp3, temp3 + 18, f_matrix_ + 36);
}

// 实现 getRMatrix 方法
auto GravComp::getRMatrix(double pose_matrix1_[9], double pose_matrix2_[9], double pose_matrix3_[9], double r_matrix_[54]) -> void {
	double temp1[18] = { 0 };
	double temp2[18] = { 0 };
	double temp3[18] = { 0 };

	getTempRMatrix(pose_matrix1_, temp1);
	getTempRMatrix(pose_matrix2_, temp2);
	getTempRMatrix(pose_matrix3_, temp3);

	std::copy(temp1, temp1 + 18, r_matrix_);
	std::copy(temp2, temp2 + 18, r_matrix_ + 18);
	std::copy(temp3, temp3 + 18, r_matrix_ + 36);
}

// 实现 getPLMatrix 方法
auto GravComp::getPLMatrix(double f_r_matrix_[54], double torque_force_data_[9], double P_L[6]) -> void {
	double U[54]{ 0 };
	double Inv_[54]{ 0 };
	double tau[9]{ 0 };

	aris::Size p[9];
	aris::Size rank;

	//   m : 行；  n : 列 
	//   rank: 可以不用写
	//   U :        m x n
	//  tau : max(m,n) x 1
	//    p : max(m,n) x 1
	//    x :        n x m

	// refer to math_matrix.hpp in aris



	// utp分解
	aris::dynamic::s_householder_utp(9, 6, f_r_matrix_, U, tau, p, rank, 1e-6);
	// 求逆
	aris::dynamic::s_householder_up2pinv(9, 6, rank, U, p, Inv_, 1e-6);
	// 矩阵乘法
	aris::dynamic::s_mm(6, 1, 9, Inv_, torque_force_data_, P_L);
}

// 实现 getCompFT 方法
auto GravComp::getCompFT(double current_pose_[16], double L_[6], double P_[6], double comp_f_[6]) -> void {
	double current_rotate[9]{ 0 };
	double inv_rotate[9]{ 0 };

	double G_vector[3]{ 0 };
	double F_vector[3]{ 0 };
	double L_vector[3]{ 0 };
	double Mass_center[3]{ 0 };
	double K_vector[3]{ 0 };

	double Mg[3]{ 0 };
	double M0[3]{ 0 };

	std::copy(L_ + 3, L_ + 6, F_vector);  // fx0 fy0 fz0
	aris::dynamic::dsp(1, 3, F_vector);
	std::copy(L_, L_ + 3, L_vector);  // lx ly lz -> Gx Gy Gz
	std::copy(P_, P_ + 3, Mass_center);  // x0 y0 z0 -> Mgx Mgy Mgz
	std::copy(P_ + 3, P_ + 6, K_vector);  // k1 k2 k3 -> Mx0 My0 Mz0

	aris::dynamic::s_pm2rm(current_pose_, current_rotate);
	aris::dynamic::s_inv_rm(current_rotate, inv_rotate);

	aris::dynamic::s_rm_dot_v3(inv_rotate, L_vector, G_vector);
	aris::dynamic::dsp(1, 3, G_vector);

	// comp x y z
	for (int i = 0; i < 3; i++) {
		comp_f_[i] = -F_vector[i] - G_vector[i];
	}

	Mg[0] = -G_vector[2] * Mass_center[1] - G_vector[1] * Mass_center[2];
	Mg[1] = -G_vector[0] * Mass_center[2] - G_vector[2] * Mass_center[0];
	Mg[2] = -G_vector[1] * Mass_center[0] - G_vector[0] * Mass_center[1];

	M0[0] = K_vector[0] - F_vector[1] * Mass_center[2] - F_vector[2] * Mass_center[1];
	M0[1] = K_vector[1] - F_vector[2] * Mass_center[0] - F_vector[1] * Mass_center[2];
	M0[2] = K_vector[2] - F_vector[0] * Mass_center[1] - F_vector[0] * Mass_center[0];

	// comp mx my mz
	for (int i = 3; i < 6; i++) {
		comp_f_[i] = -M0[i - 3] - Mg[i - 3];
	}
}
