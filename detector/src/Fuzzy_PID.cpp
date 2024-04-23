#include "Fuzzy_PID.h"
//#include "Incremental_PID.h"
#include <ros/ros.h>


void Fuzzy_PID::init(double sp, double kp, double ki, double kd)
{
    Fuzzy_PID::set_point = sp;
    Fuzzy_PID::Kp = kp;
    Fuzzy_PID::Ki = ki;
    Fuzzy_PID::Kd = kd;
}

//三角形隶属度函数
double *Fuzzy_PID::Fuzzy_input_e(double x)
{
    static double mf[7] = {0, 0, 0, 0, 0, 0, 0};
    // NB
    mf[0] = max(-2.0 - x, 0.0);
    // NM
    mf[1] = max(min(x + 3.0, -1.0 - x), 0.0);
    // NS
    mf[2] = max(min(x + 2.0, -x), 0.0);
    // ZE
    mf[3] = max(min(x + 1.0, 1.0 - x), 0.0);
    // PS
    mf[4] = max(min(x, 2.0 - x), 0.0);
    // PM
    mf[5] = max(min(x - 1.0, 3.0 - x), 0.0);
    // PB
    mf[6] = max(x - 2.0, 0.0);

    return mf;
}

//三角形隶属度函数
double *Fuzzy_PID::Fuzzy_input_ec(double x)
{
    static double mf1[7] = {0, 0, 0, 0, 0, 0, 0};
    // NB
    mf1[0] = max(-2.0 - x, 0.0);
    // NM
    mf1[1] = max(min(x + 3.0, -1.0 - x), 0.0);
    // NS
    mf1[2] = max(min(x + 2.0, -x), 0.0);
    // ZE
    mf1[3] = max(min(x + 1.0, 1.0 - x), 0.0);
    // PS
    mf1[4] = max(min(x, 2.0 - x), 0.0);
    // PM
    mf1[5] = max(min(x - 1.0, 3.0 - x), 0.0);
    // PB
    mf1[6] = max(x - 2.0, 0.0);

    return mf1;
}

double Fuzzy_PID::defuzzification(const string &type, double *L)
{
    double u = 0;
    double num = 0;
    double den = 0;
    if (type == "centroid")
    {
        for (int i = 0; i < 7; i++)
        {
            num += (i - 3) * L[i];
            den += L[i];
        }
        u = num / den;
    }
    return u;
}

double *Fuzzy_PID::Fuzzy_PID_output(double current_value, double e, double ec, double scale_P, double scale_I, double scale_D, double scale_err, double scale_d_err, Incremental_PID pid)
{
    double *input1;
    double *input2;

    e = e * scale_err;
    ec = ec * scale_d_err;
    ROS_INFO("----------------------------------------------------------------");
    ROS_INFO("e = %f", e);
    ROS_INFO("ec = %f", ec);

    input1 = Fuzzy_PID::Fuzzy_input_e(e);
    input2 = Fuzzy_PID::Fuzzy_input_ec(ec);
    double in1 = input1[0];
    double in2 = input2[0];    

    // ROS_INFO("input1 = %f, %f, %f, %f, %f, %f, %f", input1[0], input1[1], input1[2], input1[3], input1[4], input1[5], input1[6]);
    // ROS_INFO("input2 = %f, %f, %f, %f, %f, %f, %f", input2[0], input2[1], input2[2], input2[3], input2[4], input2[5], input2[6]);
        

    double fuzzy_kp[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double fuzzy_ki[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double fuzzy_kd[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            fuzzy_kp[Fuzzy_PID::rules_P[i][j]] = max(min(*(input1+i), *(input2+j)), fuzzy_kp[Fuzzy_PID::rules_P[i][j]]);
            fuzzy_ki[Fuzzy_PID::rules_I[i][j]] = max(min(*(input1+i), *(input2+j)), fuzzy_ki[Fuzzy_PID::rules_I[i][j]]);
            fuzzy_kd[Fuzzy_PID::rules_D[i][j]] = max(min(*(input1+i), *(input2+j)), fuzzy_kd[Fuzzy_PID::rules_D[i][j]]);
        }
    }
    // ROS_INFO("fuzzy_kp = %f, %f, %f, %f, %f, %f, %f", fuzzy_kp[0], fuzzy_kp[1], fuzzy_kp[2], fuzzy_kp[3], fuzzy_kp[4], fuzzy_kp[5], fuzzy_kp[6]);
    // ROS_INFO("fuzzy_ki = %f, %f, %f, %f, %f, %f, %f", fuzzy_ki[0], fuzzy_ki[1], fuzzy_ki[2], fuzzy_ki[3], fuzzy_ki[4], fuzzy_ki[5], fuzzy_ki[6]);    
    // ROS_INFO("fuzzy_kd = %f, %f, %f, %f, %f, %f, %f", fuzzy_kd[0], fuzzy_kd[1], fuzzy_kd[2], fuzzy_kd[3], fuzzy_kd[4], fuzzy_kd[5], fuzzy_kd[6]);

    double delta_P = 0.0;
    double delta_I = 0.0;
    double delta_D = 0.0;

    
   
    delta_P = Fuzzy_PID::defuzzification("centroid", fuzzy_kp);
    delta_I = Fuzzy_PID::defuzzification("centroid", fuzzy_ki);
    delta_D = Fuzzy_PID::defuzzification("centroid", fuzzy_kd);

    //ROS_INFO("delta_P = %f", delta_P * scale_P);
    //ROS_INFO("delta_I = %f", delta_I * scale_I);
    //ROS_INFO("delta_D = %f", delta_D * scale_D);


    //ROS_INFO("Fuzzy_PID::Kp: %f, Fuzzy_PID::Ki:%f, Fuzzy_PID::Kd:%f", Fuzzy_PID::Kp, Fuzzy_PID::Ki, Fuzzy_PID::Kd);    
    //ROS_INFO("----------------------------------------------------------------");
    delta_P = delta_P * scale_P;
    delta_I = delta_I * scale_I;
    delta_D = delta_D * scale_D;

    Fuzzy_PID::Kp += delta_P;
    Fuzzy_PID::Ki += delta_I;
    Fuzzy_PID::Kd += delta_D;

    //设置Kp、Ki、Kd的取值范围
    if (Fuzzy_PID::Kp > 0.8)
    {
        Fuzzy_PID::Kp = 0.8;
    }

    if (Fuzzy_PID::Kp < 0.3)
    {
        Fuzzy_PID::Kp = 0.3;
    }

    if (Fuzzy_PID::Ki > 0.025)
    {
        Fuzzy_PID::Ki = 0.025;
    }

    if (Fuzzy_PID::Ki < 0)
    {
        Fuzzy_PID::Ki = 0;
    }

    if (Fuzzy_PID::Kd > 0.6)
    {
        Fuzzy_PID::Kd = 0.6;
    }

    if (Fuzzy_PID::Kd < 0.2)
    {
        Fuzzy_PID::Kd = 0.2;
    }

	//Fuzzy_PID::Kp = 0.36;
	//Fuzzy_PID::Ki = 0.001;
	//Fuzzy_PID::Kd = 0.04;
    
    static double fpid[3]={Fuzzy_PID::Kp, Fuzzy_PID::Ki, Fuzzy_PID::Kd};
    
    //pid.init(Fuzzy_PID::set_point, Fuzzy_PID::Kp, Fuzzy_PID::Ki, Fuzzy_PID::Kd);

    //double output;
    //output = pid.Incremental_PID_output(current_value);

    //return output;
    //return pid.Incremental_PID_output(current_value);
    return fpid;
}
