#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>

#define g 9.80
// #define inf 1.0e3

using namespace std;
using namespace casadi;

class MPC
{
public:
    MPC();
    void LoadParams(const std::map<std::string, double> &params);
    Function setDynamicEquation(const Eigen::Vector3d &a_G, const Eigen::Vector4d &q_G, const Eigen::Vector3d &omega_G);
    Function setDynamicEquation();
    bool solve(const Eigen::Matrix<double, 10, 1> &current_states, const Eigen::Matrix<double, 10, 1> &desired_states, const Eigen::Vector3d &a_G, const Eigen::Vector4d &q_G, const Eigen::Vector3d &omega_G);
    bool solve(const Eigen::Matrix<double, 6, 1> &current_states, const Eigen::Matrix<double, 6, 1> &desired_states);
    // vector<double> solve(const Eigen::Matrix<double, 10, 1> &current_states, const Eigen::Matrix<double, 10, 1> &desired_states, const Eigen::Vector3d &a_G, const Eigen::Vector4d &q_G, const Eigen::Vector3d &omega_G);
    vector<double> getFirstU();

private:
    int N;
    int n_state;
    int n_input;
    double mass;
    double dt;
    double T_max, omega_max;
    double last_T, last_omega_x, last_omega_y, last_omega_z;
    double last_U_x, last_U_y, last_U_z;
    // std::map<std::string, double> params;

    Eigen::Vector3d a_G;
    Eigen::Vector4d q_G;
    Eigen::Vector3d omega_G;

    DM Q, Q_n, R, R_n;
    // MX X, U;
    SX X, U, P;
    map<string, DM> res;

    // MX state_vars, control_vars;
    Function dynamic_equation;
    unique_ptr<casadi::OptiSol> solution;
};
