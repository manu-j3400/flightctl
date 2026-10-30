#pragma once
#include <array>
#include "params.hpp"

using State = std::array<double, 12>;   // [pn pe pd u v w phi theta psi p q r]
using Control = std::array<double, 4>;  // [elevator aileron rudder throttle]

// Bit-for-bit port of matlab/dynamics_six_dof.m. Every line here has a
// corresponding line there; keep them side by side when touching either.
// This function (plus rk4_step below) is the half of the parity check
// that has to be an INDEPENDENT reimplementation, not a call back into
// MATLAB, for the cross-check to mean anything.
State dynamics_six_dof(const State& x, const Control& delta, const AircraftParams& p);

// Bit-for-bit port of matlab/rk4_step.m: classical fixed-step RK4 with
// the control held constant (zero-order hold) over the step.
State rk4_step(const State& x, const Control& delta, const AircraftParams& p, double dt);
