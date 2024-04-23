#include "mpc.h"

MPC::MPC()
{
    N = 20;

    n_state = 6;
    n_input = 3;

    dt = 0.05;
    T_max = 9.0;
    omega_max = 0.3;

    // Q = DM::zeros(10, 10);
    // Q_n = DM::zeros(10, 10);
    // R = DM::zeros(4, 4);

    Q = DM::zeros(6, 6);
    Q_n = DM::zeros(6, 6);
    R = DM::zeros(3, 3);

    last_omega_x = 0.0;
    last_omega_y = 0.0;
    last_omega_z = 0.0;
    // LoadParams(params);

    last_U_x = 0.0;
    last_U_y = 0.0;
    last_U_z = 0.0;

    // this->a_G = a_G;
    // this->q_G = q_G;
    // this->omega_G = omega_G;

    // dynamic_equation = setDynamicEquation();
}

void MPC::LoadParams(const std::map<std::string, double> &params)
{
    Q(0, 0) = params.find("w_P_xy") != params.end() ? params.at("w_P_xy") : 0;
    Q(1, 1) = params.find("w_P_xy") != params.end() ? params.at("w_P_xy") : 0;
    Q(2, 2) = params.find("w_P_z") != params.end() ? params.at("w_P_z") : 0;
    Q(3, 3) = params.find("w_V_xy") != params.end() ? params.at("w_V_xy") : 0;
    Q(4, 4) = params.find("w_V_xy") != params.end() ? params.at("w_V_xy") : 0;
    Q(5, 5) = params.find("w_V_z") != params.end() ? params.at("w_V_z") : 0;
    // Q(6, 6) = params.find("w_q") != params.end() ? params.at("w_q") : 0;
    // Q(7, 7) = params.find("w_q") != params.end() ? params.at("w_q") : 0;
    // Q(8, 8) = params.find("w_q") != params.end() ? params.at("w_q") : 0;
    // Q(9, 9) = params.find("w_q") != params.end() ? params.at("w_q") : 0;

    /*
    Q_n(0, 0) = params.find("w_P_xy_n") != params.end() ? params.at("w_P_xy_n") : 0;
    Q_n(1, 1) = params.find("w_P_xy_n") != params.end() ? params.at("w_P_xy_n") : 0;
    Q_n(2, 2) = params.find("w_P_z_n") != params.end() ? params.at("w_P_z_n") : 0;
    Q_n(3, 3) = params.find("w_V_xy_n") != params.end() ? params.at("w_V_xy_n") : 0;
    Q_n(4, 4) = params.find("w_V_xy_n") != params.end() ? params.at("w_V_xy_n") : 0;
    Q_n(5, 5) = params.find("w_V_z_n") != params.end() ? params.at("w_V_z_n") : 0;
    Q_n(6, 6) = params.find("w_q_n") != params.end() ? params.at("w_q_n") : 0;
    Q_n(7, 7) = params.find("w_q_n") != params.end() ? params.at("w_q_n") : 0;
    Q_n(8, 8) = params.find("w_q_n") != params.end() ? params.at("w_q_n") : 0;
    Q_n(9, 9) = params.find("w_q_n") != params.end() ? params.at("w_q_n") : 0;
    */

    Q_n(0, 0) = params.find("w_P_xy_n") != params.end() ? params.at("w_P_xy_n") : 0;
    Q_n(1, 1) = params.find("w_P_xy_n") != params.end() ? params.at("w_P_xy_n") : 0;
    Q_n(2, 2) = params.find("w_P_z_n") != params.end() ? params.at("w_P_z_n") : 0;
    Q_n(3, 3) = params.find("w_V_xy_n") != params.end() ? params.at("w_V_xy_n") : 0;
    Q_n(4, 4) = params.find("w_V_xy_n") != params.end() ? params.at("w_V_xy_n") : 0;
    Q_n(5, 5) = params.find("w_V_z_n") != params.end() ? params.at("w_V_z_n") : 0;

    // R(0, 0) = params.find("w_T") != params.end() ? params.at("w_T") : 0;
    // R(1, 1) = params.find("w_omega_x") != params.end() ? params.at("w_omega_x") : 0;
    // R(2, 2) = params.find("w_omega_y") != params.end() ? params.at("w_omega_y") : 0;
    // R(3, 3) = params.find("w_omega_z") != params.end() ? params.at("w_omega_z") : 0;

    R(0, 0) = params.find("w_U_x") != params.end() ? params.at("w_U_x") : 0;
    R(1, 1) = params.find("w_U_y") != params.end() ? params.at("w_U_y") : 0;
    R(2, 2) = params.find("w_U_z") != params.end() ? params.at("w_U_z") : 0;

    mass = params.find("mass") != params.end() ? params.at("mass") : 1.5;
    last_T = g / mass;
}

/*
bool MPC::solve(const Eigen::Matrix<double, 10, 1> &current_states, const Eigen::Matrix<double, 10, 1> &desired_states, const Eigen::Vector3d &a_G, const Eigen::Vector4d &q_G, const Eigen::Vector3d &omega_G)
{

    // Opti opti = Opti();
    Opti opti = Opti("conic");

    Slice all;

    MX cost = 0;
    X = opti.variable(10, N);
    U = opti.variable(4, N - 1);

    // MX P_x = X(0, all);
    // MX P_y = X(1, all);
    // MX P_z = X(2, all);
    // MX V_x = X(3, all);
    // MX V_y = X(4, all);
    // MX V_z = X(5, all);
    // MX q_w = X(6, all);
    // MX q_x = X(7, all);
    // MX q_y = X(8, all);
    // MX q_z = X(9, all);

    MX T = U(0, all);
    MX omega_x = U(1, all);
    MX omega_y = U(2, all);
    MX omega_z = U(3, all);

    MX X_ref = opti.parameter(10);
    MX X_cur = opti.parameter(10);
    DM x_tmp = {current_states[0], current_states[1], current_states[2], current_states[3], current_states[4], current_states[5], current_states[6], current_states[7], current_states[8], current_states[9]};
    DM X_ref_d = {desired_states[0], desired_states[1], desired_states[2], desired_states[3], desired_states[4], desired_states[5], desired_states[6], desired_states[7], desired_states[8], desired_states[9]};

    // cout << "current_state: " << current_states[0] << ", " << current_states[1] << ", " << current_states[2] << ", " << current_states[3] << ", " << current_states[4] << ", " << current_states[5] << ", " << current_states[6] << ", " << current_states[7] << ", " << current_states[8] << ", " << current_states[9] << endl;

    // cout << "desired_state: " << desired_states[0] << ", " << desired_states[1] << ", " << desired_states[2] << ", " << desired_states[3] << ", " << desired_states[4] << ", " << desired_states[5] << ", " << desired_states[6] << ", " << desired_states[7] << ", " << desired_states[8] << ", " << desired_states[9] << endl;

    opti.set_value(X_cur, x_tmp);
    X_ref = MX::reshape(X_ref_d, 10, 1);
    // cout << "set current state success" << endl;
    // cout << X_ref << endl;

    // 按列索引

    // vector<double> X_ref_v(desired_states.data(), desired_states.data() + desired_states.size());
    // vector<double> X_ref_tmp(X_ref_v);

    // for (int i = 0; i < N - 1; i++)
    //{
    //     X_ref_v.insert(X_ref_v.end(), X_ref_tmp.begin(), X_ref_tmp.end());
    // }

    // DM X_ref_d(X_ref_v);
    // X_ref = MX::reshape(X_ref_d, 10, N);

    // cout << "Q: " << Q << endl;
    // cout << "Q_n: " << Q_n << endl;
    // cout << "R: " << R << endl;

    // set cost function
    for (int i = 0; i < N - 1; i++)
    {
        MX X_err = X(all, i) - X_ref(all);
        MX U_0 = U(all, i);
        // cout << "U_0 size:" << U_0.size() << endl;
        // cout << "cost size:" << cost_.size() << endl;
        cost += MX::mtimes({X_err.T(), Q, X_err});
        // cout << "cost size:" << cost_.size() << endl;
        cost += MX::mtimes({U_0.T(), R, U_0});
        // cout << "cost size:" << cost_.size() << endl;
    }

    cost += MX::mtimes({(X(all, N - 1) - X_ref(all)).T(), Q_n,
                        X(all, N - 1) - X_ref(all)});
    opti.minimize(cost);

    dynamic_equation = setDynamicEquation(a_G, q_G, omega_G);
    // dynamic_equation = Function("dynamic_equation", {state_vars, control_vars}, {rhs});

    // kinematic constrains
    for (int i = 0; i < N - 1; i++)
    {
        vector<MX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);
        MX X_next = dynamic_equation(input).at(0) * dt + X(all, i);
        opti.subject_to(X_next == X(all, i + 1));
    }

    // init value
    opti.subject_to(X(all, 0) == X_cur);

    opti.subject_to(0 <= T <= T_max);
    opti.subject_to(-omega_max <= omega_x <= omega_max);
    opti.subject_to(-omega_max <= omega_y <= omega_max);
    opti.subject_to(-omega_max <= omega_z <= omega_max);

    // set solver
    string solver_name = "osqp";
    casadi::Dict solver;

    solver["expand"] = true;
    solver["print_time"] = 0;

    if (solver_name == "ipopt")
    {
        solver["ipopt.max_iter"] = 50;
        solver["ipopt.max_cpu_time"] = 0.02;
        solver["ipopt.print_level"] = 0;
        solver["ipopt.acceptable_tol"] = 1e-3;
        solver["ipopt.acceptable_obj_change_tol"] = 1e-3;
        solver["ipopt.linear_solver"] = "ma27";
    }
    else if (solver_name == "qpoases")
    {
        solver["printLevel"] = "none";
        // solver["nWSR"] = 50;
        solver["CPUtime"] = 0.02;
        solver["sparse"] = true;
    }
    else if (solver_name == "osqp")
    {
        solver["verbose"] = false;
    }

    opti.solver(solver_name, solver);

    // auto sol = opti.solve();
    solution = std::make_unique<casadi::OptiSol>(opti.solve());

    return true;
}

vector<double> MPC::getFirstU()
{
    vector<double> vec;
    auto u1 = solution->value(U)(0, 0);
    auto u2 = solution->value(U)(1, 0);
    auto u3 = solution->value(U)(2, 0);
    auto u4 = solution->value(U)(3, 0);

    vec.emplace_back(static_cast<double>(u1));
    vec.emplace_back(static_cast<double>(u2));
    vec.emplace_back(static_cast<double>(u3));
    vec.emplace_back(static_cast<double>(u4));
    return vec;
}
*/

/*
Function MPC::setDynamicEquation(const Eigen::Vector3d &a_G, const Eigen::Vector4d &q_G, const Eigen::Vector3d &omega_G)
{

    SX P_x = SX::sym("P_x");
    SX P_y = SX::sym("P_y");
    SX P_z = SX::sym("P_z");
    SX V_x = SX::sym("V_x");
    SX V_y = SX::sym("V_y");
    SX V_z = SX::sym("V_z");
    SX q_w = SX::sym("q_w");
    SX q_x = SX::sym("q_x");
    SX q_y = SX::sym("q_y");
    SX q_z = SX::sym("q_z");
    SX state_vars = SX::vertcat({P_x, P_y, P_z, V_x, V_y, V_z, q_w, q_x, q_y, q_z});

    SX T = SX::sym("T");
    SX omega_x = SX::sym("omega_x");
    SX omega_y = SX::sym("omega_y");
    SX omega_z = SX::sym("omega_z");
    SX control_vars = SX::vertcat({T, omega_x, omega_y, omega_z});

    SX a_G_x = a_G(0);
    SX a_G_y = a_G(1);
    SX a_G_z = a_G(2);

    SX q_G_w = q_G(0);
    SX q_G_x = q_G(1);
    SX q_G_y = q_G(2);
    SX q_G_z = q_G(3);

    SX omega_G_x = omega_G(0);
    SX omega_G_y = omega_G(1);
    SX omega_G_z = omega_G(2);

    SX rhs = SX::vertcat({V_x,
                          V_y,
                          V_z,
                          -(-omega_G_z * V_y + omega_G_y * V_z) + 2 * (q_x * q_z + q_w * q_y) * T - 2 * (q_G_x * q_G_z - q_G_w * q_G_y) * g,
                          -(omega_G_z * V_x - omega_G_x * V_z) + 2 * (q_y * q_z - q_w * q_x) * T - 2 * (q_G_y * q_G_z + q_G_w * q_G_x) * g,
                          -(-omega_G_y * V_x + omega_G_x * V_y) + (1 - 2 * q_x * q_x - 2 * q_y * q_y) * T - (1 - 2 * q_G_x * q_G_x - 2 * q_G_y * q_G_y) * g,
                          0.5 * (-omega_x * q_x - omega_y * q_y - omega_z * q_z + omega_G_x * q_x + omega_G_y * q_y + omega_G_z * q_z),
                          0.5 * (omega_x * q_w + omega_z * q_y - omega_y * q_z - omega_G_x * q_w - omega_G_z * q_y + omega_G_y * q_z),
                          0.5 * (omega_y * q_w - omega_z * q_x + omega_x * q_z - omega_G_y * q_w + omega_G_z * q_x - omega_G_x * q_z),
                          0.5 * (omega_z * q_w + omega_y * q_x - omega_x * q_y - omega_G_z * q_w - omega_G_y * q_x + omega_G_x * q_y)});
    // -a_G_x + 2 * (q_x * q_z + q_w * q_y) * T - 2 * (q_G_x * q_G_z - q_G_w * q_G_y) * g;
    // -a_G_y + 2 * (q_y * q_z - q_w * q_x) * T - 2 * (q_G_y * q_G_z + q_G_w * q_G_x) * g;
    // -a_G_z + (1 - 2 * q_x * q_x - 2 * q_y * q_y) * T - (1 - 2 * q_G_w * q_G_w - 2 * q_G_y * q_G_y) * g;
    // 0.5 * (omega_G_x * q_x + omega_G_y * q_y + omega_G_z * q_z - q_x * omega_x - q_y * omega_y - q_z * omega_z);
    // 0.5 * (omega_G_z * q_y - omega_G_x * q_w - omega_G_y * q_z + q_w * omega_x + q_y * omega_z - q_z * omega_y);
    // 0.5 * (omega_G_x * q_z - omega_G_z * q_x - omega_G_y * q_w + q_w * omega_y - q_x * omega_z + q_z * omega_x);
    // 0.5 * (omega_G_y * q_x - omega_G_z * q_w - omega_G_x * q_y + q_w * omega_z + q_x * omega_y - q_y * omega_x);

    // cout << "rhs: " << rhs << endl;
    return Function("dynamic_equation", {state_vars, control_vars}, {rhs});
}
*/

/*
bool MPC::solve(const Eigen::Matrix<double, 10, 1> &current_states, const Eigen::Matrix<double, 10, 1> &desired_states, const Eigen::Vector3d &a_G, const Eigen::Vector4d &q_G, const Eigen::Vector3d &omega_G)
{
    Slice all;

    X = SX::sym("X", 10, N + 1);
    U = SX::sym("U", 4, N);

    SX obj = 0;
    SX constraints;

    SX X_ref = {desired_states[0], desired_states[1], desired_states[2], desired_states[3], desired_states[4], desired_states[5], desired_states[6], desired_states[7], desired_states[8], desired_states[9]};
    SX X_cur = {current_states[0], current_states[1], current_states[2], current_states[3], current_states[4], current_states[5], current_states[6], current_states[7], current_states[8], current_states[9]};

    // cout << "desired_states: " << desired_states.transpose() << endl;
    // cout << "current_states: " << current_states.transpose() << endl;
    //  cout << "X_ref: " << X_ref << endl;
    //  cout << "X_cur: " << X_cur << endl;

    constraints = X(all, 0) - X_cur;
    constraints = SX::reshape(constraints, -1, 1);

    dynamic_equation = setDynamicEquation(a_G, q_G, omega_G);
    for (int i = 0; i < N; i++)
    {
        obj += SX::mtimes(SX::mtimes((X(all, i) - X_ref).T(), Q), X(all, i) - X_ref);
        obj += SX::mtimes(SX::mtimes(U(all, i).T(), R), U(all, i));

        vector<SX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);
        SX X_next = dynamic_equation(input).at(0) * dt + X(all, i);
        constraints = SX::vertcat({constraints, X_next - X(all, i + 1)});
    }

    obj += SX::mtimes(SX::mtimes((X(all, N) - X_ref(all)).T(), Q_n), (X(all, N) - X_ref(all)));

    SXDict qp = {{"x", SX::vertcat({SX::reshape(X, -1, 1), SX::reshape(U, -1, 1)})}, {"f", obj}, {"g", constraints}};

    string solver_name = "osqp";
    // string solver_name = "qpoases";
    casadi::Dict solver_opts;

    solver_opts["expand"] = true;
    solver_opts["print_time"] = 0;

    if (solver_name == "ipopt")
    {
        solver_opts["ipopt.max_iter"] = 50;
        solver_opts["ipopt.max_cpu_time"] = 0.02;
        solver_opts["ipopt.print_level"] = 0;
        solver_opts["ipopt.acceptable_tol"] = 1e-3;
        solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-3;
        solver_opts["ipopt.linear_solver"] = "ma27";
    }
    else if (solver_name == "qpoases")
    {
        solver_opts["printLevel"] = "none";
        // solver_opts["nWSR"] = 50;
        solver_opts["CPUtime"] = 0.02;
        solver_opts["sparse"] = true;
    }
    else if (solver_name == "osqp")
    {
        // solver_opts["verbose"] = true;
        solver_opts["warm_start_primal"] = true;
    }

    Function solver = qpsol("qpsol", solver_name, qp, solver_opts);

    vector<double> lbg;
    vector<double> ubg;
    vector<double> lbx;
    vector<double> ubx;

    for (int i = 0; i < N + 1; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            lbg.emplace_back(0.0);
            ubg.emplace_back(0.0);
        }
    }

    for (int i = 0; i < N + 1; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            lbx.emplace_back(-casadi::inf);
            ubx.emplace_back(casadi::inf);
        }
    }

    for (int i = 0; i < N; i++)
    {
        lbx.emplace_back(1.0);
        ubx.emplace_back(T_max);
        lbx.emplace_back(-omega_max);
        ubx.emplace_back(omega_max);
        lbx.emplace_back(-omega_max);
        ubx.emplace_back(omega_max);
        lbx.emplace_back(-omega_max);
        ubx.emplace_back(omega_max);
    }

    vector<double> init_values;
    vector<double> X_init((N + 1) * 10, 0.0);
    vector<double> U_init(N * 4, 0.0);
    for (int i = 0; i < 10; i++)
    {
        X_init[i] = current_states(i);
    }
    U_init[0] = last_T;
    U_init[1] = last_omega_x;
    U_init[2] = last_omega_y;
    U_init[3] = last_omega_z;

    // U_init[0] = g / mass;
    // U_init[1] = 0.0;
    // U_init[2] = 0.0;
    // U_init[3] = 0.0;

    std::merge(X_init.begin(), X_init.end(), U_init.begin(), U_init.end(), std::back_inserter(init_values));

    DMDict arg = {{"lbx", lbx}, {"ubx", ubx}, {"lbg", lbg}, {"ubg", ubg}, {"x0", init_values}};
    res = solver(arg);

    return true;
}

/*
vector<double> MPC::getFirstU()
{
    vector<double> result_x(res.at("x"));
    vector<double> vec;
    vec.assign(result_x.begin() + (N + 1) * 10, result_x.begin() + (N + 1) * 10 + 4);

    last_T = vec[0];
    last_omega_x = vec[1];
    last_omega_y = vec[2];
    last_omega_z = vec[3];

    return vec;
}
*/

Function MPC::setDynamicEquation()
{

    SX P_x = SX::sym("P_x");
    SX P_y = SX::sym("P_y");
    SX P_z = SX::sym("P_z");
    SX V_x = SX::sym("V_x");
    SX V_y = SX::sym("V_y");
    SX V_z = SX::sym("V_z");
    SX state_vars = SX::vertcat({P_x, P_y, P_z, V_x, V_y, V_z});

    SX U_x = SX::sym("U_x");
    SX U_y = SX::sym("U_y");
    SX U_z = SX::sym("U_z");
    SX control_vars = SX::vertcat({U_x, U_y, U_z});

    SX rhs = SX::vertcat({V_x,
                          V_y,
                          V_z,
                          U_x,
                          U_y,
                          U_z});

    return Function("dynamic_equation", {state_vars, control_vars}, {rhs});
}

bool MPC::solve(const Eigen::Matrix<double, 6, 1> &current_states, const Eigen::Matrix<double, 6, 1> &desired_states)
{
    Slice all;

    X = SX::sym("X", 6, N + 1);
    U = SX::sym("U", 3, N);
    P = SX::sym("P", 12);

    SX obj = 0;
    SX constraints;

    SX X_ref = {desired_states[0], desired_states[1], desired_states[2], desired_states[3], desired_states[4], desired_states[5]};
    SX X_cur = {current_states[0], current_states[1], current_states[2], current_states[3], current_states[4], current_states[5]};

    // cout << "desired_states: " << desired_states.transpose() << endl;
    // cout << "current_states: " << current_states.transpose() << endl;
    // cout << "X_ref: " << X_ref << endl;
    // cout << "X_cur: " << X_cur << endl;

    constraints = X(all, 0) - P(Slice(0, 6));
    constraints = SX::reshape(constraints, -1, 1);

    // cout << "Q: " << Q << endl;
    // cout << "Q_n: " << Q_n << endl;
    // cout << "R: " << R << endl;

    dynamic_equation = setDynamicEquation();
    for (int i = 0; i < N; i++)
    {
        obj += SX::mtimes(SX::mtimes((X(all, i) - P(Slice(6, 12))).T(), Q), (X(all, i) - P(Slice(6, 12))));
        obj += SX::mtimes(SX::mtimes(U(all, i).T(), R), U(all, i));

        vector<SX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);
        auto X_next = dynamic_equation(input).at(0) * dt + X(all, i);
        constraints = SX::vertcat({constraints, X_next - X(all, i + 1)});
    }

    obj += SX::mtimes(SX::mtimes((X(all, N) - P(Slice(6, 12))).T(), Q_n), (X(all, N) - P(Slice(6, 12))));

    SXDict qp = {{"x", SX::vertcat({SX::reshape(X, -1, 1), SX::reshape(U, -1, 1)})}, {"f", obj}, {"p", P}, {"g", constraints}};

    string solver_name = "osqp";
    // string solver_name = "qpoases";
    casadi::Dict solver_opts;

    solver_opts["expand"] = true;
    solver_opts["print_time"] = 0;

    if (solver_name == "ipopt")
    {
        solver_opts["ipopt.max_iter"] = 50;
        solver_opts["ipopt.max_cpu_time"] = 0.02;
        solver_opts["ipopt.print_level"] = 0;
        solver_opts["ipopt.acceptable_tol"] = 1e-3;
        solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-3;
        solver_opts["ipopt.linear_solver"] = "ma27";
    }
    else if (solver_name == "qpoases")
    {
        solver_opts["printLevel"] = "none";
        // solver_opts["nWSR"] = 50;
        solver_opts["CPUtime"] = 0.02;
        solver_opts["sparse"] = true;
    }
    else if (solver_name == "osqp")
    {
        // solver_opts["verbose"] = true;
        solver_opts["warm_start_primal"] = true;
    }

    Function solver = qpsol("qpsol", solver_name, qp, solver_opts);

    vector<double> lbg;
    vector<double> ubg;
    vector<double> lbx;
    vector<double> ubx;

    for (int i = 0; i < N + 1; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            lbg.emplace_back(0.0);
            ubg.emplace_back(0.0);
        }
    }

    for (int i = 0; i < N + 1; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            lbx.emplace_back(-casadi::inf);
            ubx.emplace_back(casadi::inf);
        }
    }

    for (int i = 0; i < N; i++)
    {
        lbx.emplace_back(-5.0);
        ubx.emplace_back(5.0);
        lbx.emplace_back(-5.0);
        ubx.emplace_back(5.0);
        lbx.emplace_back(-2.0);
        ubx.emplace_back(2.0);
    }

    vector<double> init_values;
    vector<double> control_params;
    vector<double> X_init((N + 1) * 6, 0.0);
    vector<double> U_init(N * 3, 0.0);
    vector<double> X_0{current_states[0], current_states[1], current_states[2], current_states[3], current_states[4], current_states[5]};
    vector<double> X_N{desired_states[0], desired_states[1], desired_states[2], desired_states[3], desired_states[4], desired_states[5]};

    for (int i = 0; i < 6; i++)
    {
        X_init[i] = current_states[i];
    }
    U_init[0] = last_U_x;
    U_init[1] = last_U_y;
    U_init[2] = last_U_z;

    // U_init[0] = 0.0;
    // U_init[1] = 0.0;
    // U_init[2] = 0.0;

    // cout << "X_init: " << X_init << endl;
    // cout << "U_init: " << U_init << endl;
    // cout << "X_0: " << X_0 << endl;
    // cout << "X_N: " << X_N << endl;

    init_values.insert(init_values.end(), X_init.begin(), X_init.end());
    init_values.insert(init_values.end(), U_init.begin(), U_init.end());

    control_params.insert(control_params.end(), X_0.begin(), X_0.end());
    control_params.insert(control_params.end(), X_N.begin(), X_N.end());

    // std::merge(X_init.begin(), X_init.end(), U_init.begin(), U_init.end(), std::back_inserter(init_values));
    // std::merge(X_0.begin(), X_0.end(), X_N.begin(), X_N.end(), std::back_inserter(control_params));
    // cout << "init_values: " << init_values << endl;
    // cout << "control_params: " << control_params << endl;

    DMDict arg = {{"lbx", lbx}, {"ubx", ubx}, {"lbg", lbg}, {"ubg", ubg}, {"p", control_params}, {"x0", init_values}};
    res = solver(arg);

    return true;
}

vector<double> MPC::getFirstU()
{
    vector<double> result_x(res.at("x"));
    // cout << "result_x: " << result_x << endl;
    vector<double> vec;
    vec.assign(result_x.begin() + (N + 1) * 6, result_x.begin() + (N + 1) * 6 + 3);

    last_U_x = vec[0];
    last_U_y = vec[1];
    last_U_z = vec[2];

    return vec;
}