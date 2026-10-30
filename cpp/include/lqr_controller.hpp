#pragma once
#include <array>
#include <string>
#include "dynamics.hpp"

// 4x12 gain matrix, row-major: row i = [elevator|aileron|rudder|throttle]
// gain for state j. Zero-padded on the (pn,pe,pd,psi) columns, same
// convention as matlab/design_lqr.m.
using GainMatrix = std::array<std::array<double, 12>, 4>;

struct LqrController {
    State x_trim{};
    Control delta_trim{};
    GainMatrix K{};
    AircraftParams params;

    Control operator()(const State& x) const;
};

// Loads x_trim, delta_trim and K from a "case*_setup.csv" written by
// run_design_and_export.m / evaluate_compare.m.
LqrController load_lqr_controller(const std::string& setup_csv, const AircraftParams& params);
