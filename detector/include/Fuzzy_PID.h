#include <iostream>
#include <string>
#include <math.h>
#include <algorithm>
#include "Incremental_PID.h"

using std::string;
using namespace std;

#define N_B 0
#define N_M 1
#define N_S 2
#define Z_E 3
#define P_S 4
#define P_M 5
#define P_B 6

class Fuzzy_PID
{
private:
    double Kp;
    double Ki;
    double Kd;
    double set_point;
    double scale_P;
    double scale_I;
    double scale_D;
    double scale_err;
    double scale_d_err;

public:
    int rules_P[7][7] = {{P_B, P_B, P_M, P_M, P_S, Z_E, Z_E},
                         {P_B, P_B, P_M, P_S, P_S, Z_E, N_S},
                         {P_M, P_M, P_M, P_S, Z_E, N_S, N_S},
                         {P_M, P_M, P_S, Z_E, N_S, N_M, N_M},
                         {P_S, P_S, Z_E, N_S, N_S, N_M, N_M},
                         {P_S, Z_E, N_S, N_M, N_M, N_M, N_B},
                         {Z_E, Z_E, N_M, N_M, N_M, N_B, N_B}};
    
    int rules_D[7][7] = {{N_B, N_B, N_M, N_M, N_S, Z_E, Z_E},
                         {N_B, N_B, N_M, N_S, N_S, Z_E, Z_E},
                         {N_B, N_M, N_S, N_S, Z_E, P_S, P_S},
                         {N_M, N_M, N_S, Z_E, P_S, P_M, P_M},
                         {N_M, N_S, Z_E, P_S, P_S, P_M, P_B},
                         {Z_E, Z_E, P_S, P_S, P_M, P_B, P_B},
                         {Z_E, Z_E, P_S, P_M, P_M, P_B, P_B}};

    int rules_I[7][7] = {{P_S, N_S, N_B, N_B, N_B, N_M, P_S},
                         {P_S, N_S, N_B, N_M, N_M, N_S, Z_E},
                         {Z_E, N_S, N_M, N_M, N_S, N_S, Z_E},
                         {Z_E, N_S, N_S, N_S, N_S, N_S, Z_E},
                         {Z_E, Z_E, Z_E, Z_E, Z_E, Z_E, Z_E},
                         {P_B, N_S, P_S, P_S, P_S, P_S, P_B},
                         {P_B, P_M, P_M, P_M, P_S, P_S, P_B}};

    void init(double sp, double kp, double ki, double kd);
    double *Fuzzy_input_e(double x);
    double *Fuzzy_input_ec(double x);
    double defuzzification(const string &type, double *L);
    double *Fuzzy_PID_output(double current_value, double e, double ec, double scale_P, double scale_I, double scale_D, double scale_err, double scale_d_err,Incremental_PID pid);
};
