// flightctl_cpp -- real-time C++ implementation of the closed-loop
// nonlinear 6-DOF flight dynamics, driven by either the LQR gain or the
// trained NN controller exported from MATLAB. Runs the IDENTICAL
// fixed-step RK4 loop as matlab/run_design_and_export.m /
// evaluate_compare.m so the output trajectory can be diffed against the
// MATLAB one for the verification harness, and reports real-time
// computation headroom (how much of the control-loop time budget the
// controller + dynamics integration actually consumes).
//
// Usage:
//   flightctl_cpp lqr <aircraft_params.csv> <caseN_setup.csv> - <out.csv>
//   flightctl_cpp nn  <aircraft_params.csv> <caseN_setup.csv> <nn_weights.csv> <out.csv>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

#include "csv_utils.hpp"
#include "dynamics.hpp"
#include "lqr_controller.hpp"
#include "nn_controller.hpp"

static State read_state_vec(const std::string& path, const std::string& label) {
    auto v = read_labeled_vector_csv(path, label);
    State s{};
    for (int i = 0; i < 12; ++i) s[i] = v[i];
    return s;
}

int main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr << "usage: " << argv[0] << " <lqr|nn> <params.csv> <setup.csv> <weights.csv|-> <out.csv>\n";
        return 1;
    }
    const std::string mode = argv[1];
    const std::string params_csv = argv[2];
    const std::string setup_csv = argv[3];
    const std::string weights_csv = argv[4];
    const std::string out_csv = argv[5];

    AircraftParams params = load_aircraft_params(params_csv);

    const double dt = read_labeled_scalar_csv(setup_csv, "dt");
    const int n_steps = static_cast<int>(read_labeled_scalar_csv(setup_csv, "n_steps"));
    State x_trim = read_state_vec(setup_csv, "x_trim");
    State x0 = read_state_vec(setup_csv, "x0");

    auto delta_trim_v = read_labeled_vector_csv(setup_csv, "delta_trim");
    Control delta_trim{};
    for (int i = 0; i < 4; ++i) delta_trim[i] = delta_trim_v[i];

    LqrController lqr;
    NnController nn;
    bool use_nn = (mode == "nn");
    if (use_nn) {
        nn = load_nn_controller(weights_csv, x_trim, params);
    } else if (mode == "lqr") {
        lqr = load_lqr_controller(setup_csv, params);
    } else {
        std::cerr << "unknown mode '" << mode << "' (expected lqr or nn)\n";
        return 1;
    }

    std::ofstream out(out_csv);
    if (!out.is_open()) {
        std::cerr << "could not open output file " << out_csv << "\n";
        return 1;
    }
    out << "t,pn,pe,pd,u,v,w,phi,theta,psi,p,q,r,de,da,dr,dt_thr\n";
    out.precision(17);

    auto write_row = [&](double t, const State& x, const Control& u) {
        out << t;
        for (double v : x) out << "," << v;
        for (double v : u) out << "," << v;
        out << "\n";
    };

    // Row j always logs u = ctrl(x) EVALUATED AT THAT SAME ROW'S STATE,
    // matching matlab/rollout_cost.m's convention exactly (there, U(:,j)
    // is literally ctrl_fn(X(:,j)) for every j, including the final row,
    // which is a fresh post-loop evaluation). Logging u BEFORE stepping
    // x (rather than logging the u that produced the just-written x) is
    // what makes this match MATLAB bit-for-bit instead of off by one
    // sample -- an earlier version of this loop logged the pre-step u
    // alongside the post-step x, which left every physical STATE
    // matching MATLAB to 1e-16 (the integration itself was always
    // correct) while every logged CONTROL was one step stale, exactly
    // the discrepancy verification/compare_trajectories.m caught.
    State x = x0;
    double total_ctrl_ns = 0.0, total_dyn_ns = 0.0;
    double max_ctrl_ns = 0.0, max_dyn_ns = 0.0;

    for (int j = 0; j <= n_steps; ++j) {
        auto t0 = std::chrono::steady_clock::now();
        Control u = use_nn ? nn(x) : lqr(x);
        auto t1 = std::chrono::steady_clock::now();

        write_row(j * dt, x, u);

        if (j < n_steps) {
            x = rk4_step(x, u, params, dt);
            auto t2 = std::chrono::steady_clock::now();
            double ctrl_ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
            double dyn_ns  = std::chrono::duration<double, std::nano>(t2 - t1).count();
            total_ctrl_ns += ctrl_ns; total_dyn_ns += dyn_ns;
            max_ctrl_ns = std::max(max_ctrl_ns, ctrl_ns);
            max_dyn_ns  = std::max(max_dyn_ns, dyn_ns);
        }
    }
    out.close();

    const double dt_budget_ns = dt * 1e9;
    const double mean_total_ns = (total_ctrl_ns + total_dyn_ns) / n_steps;
    const double max_total_ns = max_ctrl_ns + max_dyn_ns; // conservative (worst control + worst dynamics step, not necessarily same step)

    std::fprintf(stderr,
        "[%s] %d steps, dt=%.6f s (%.1f Hz budget = %.0f ns/step)\n"
        "  mean compute/step : %.0f ns  (controller %.0f ns, dynamics %.0f ns)\n"
        "  worst compute/step: %.0f ns (upper bound)\n"
        "  real-time headroom: %.1fx (budget / mean compute)\n",
        mode.c_str(), n_steps, dt, 1.0/dt, dt_budget_ns,
        mean_total_ns, total_ctrl_ns/n_steps, total_dyn_ns/n_steps,
        max_total_ns,
        dt_budget_ns / mean_total_ns);

    std::fprintf(stderr, "wrote %s\n", out_csv.c_str());
    return 0;
}
