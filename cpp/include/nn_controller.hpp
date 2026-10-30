#pragma once
#include <array>
#include <string>
#include <vector>
#include "dynamics.hpp"

// Bit-for-bit port of matlab/nn_policy.m's forward() + matlab/apply_control_nn.m.
// Architecture is fixed at 8->24->24->4 to match the trained weights file;
// if you retrain with different layer sizes in MATLAB, update kLayerSizes
// here to match (the loader checks the parameter count and throws if they
// don't agree, so a silent mismatch isn't possible).
struct NnController {
    static constexpr std::array<int, 4> kLayerSizes = {8, 24, 24, 4};

    std::vector<double> theta;        // flattened [W1 b1 W2 b2 W3 b3]
    std::array<double, 8> scale{};
    State x_trim{};
    AircraftParams params;

    Control operator()(const State& x) const;
};

NnController load_nn_controller(const std::string& weights_csv, const State& x_trim, const AircraftParams& params);
