
const double PI = 3.141592653589793;


class cosCurve
{
private:
    double a_;
    double w_;
    double p_;

public:
    auto getCurve(int count) -> double;  
    cosCurve(double a, double w, double p)
    {
        a_ = a;
        w_ = w;
        p_ = p;
    }  
    ~cosCurve() {} 
};



///功能：生成0->1的梯形曲线。可根据输入的加速度和速度判断曲线为梯形还是三角形
//   ##参数定义##
//  Tc:生成曲线总共需要的时间，由输入的加速度和速度计算
//   v:速度，由用户输入，构造函数初始化
//   a:加速度，由用户输入，构造函数初始化
//  ta:加速段所需的时间，由输入的速度和加速度计算得到
class TCurve
{
private:
    double Tc_;
    double v_;
    double a_;
    double ta_;

public:
    auto getTCurve(int count) -> double;
    auto getCurveParam() -> void;
    auto getTc() -> double { return Tc_; };
    TCurve(double a, double v) { a_ = a; v_ = v; }
    ~TCurve() {}
};


//T形曲线改，输入值为（最大加速度，最大速度，目标位置）
//tc_指运动到目标完成时间；
//tm_指以最大速度运动所需时间；
//v_为最大运动速度；
//a_为最大运动加速度；
//ta_指加速时间；
//p_指最终运动位置；
//pa_指加减速所需要运动的位置。
class TCurve2
{
private:
    double tc_;
    double tm_;
    double v_;
    double a_;
    double ta_;
    double p_;
    double pa_;

public:
    auto getTCurve(int count) -> double;
    auto getCurveParam() -> void;
    auto getTc() -> double { return tc_; };
    TCurve2(double a, double v, double p) { a_ = a; v_ = v; p_ = p; }
    ~TCurve2() {}
};






class GravComp
{
private:


public:
    auto getTorqueVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double torque_vector_[9]) -> void;
    auto getForceVector(double force_data1_[6], double force_data2_[6], double force_data3_[6], double force_vector_[9]) -> void;
    auto getTempFMatrix(double force_data_[6], double temp_[18]) -> void;
    auto getTempRMatrix(double pose_matrix_[9], double temp_[18]) -> void;
    auto getFMatrix(double force_data1_[6], double force_data2_[6], double force_data3_[6], double f_matrix_[54]) -> void;
    auto getRMatrix(double pose_matrix1_[9], double pose_matrix2_[9], double pose_matrix3_[9], double r_matrix_[54]) -> void;
    auto getPLMatrix(double f_r_matrix_[54], double torque_force_data_[9], double P_L[6]) -> void;
    auto getCompFT(double current_pose_[16], double L_[6], double P_[6], double comp_f_[6]) -> void;

};

