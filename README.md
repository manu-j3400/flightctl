# flightctl

A nonlinear 6-DOF fixed-wing flight control system, designed and verified in
MATLAB/Octave, with an independently-written, verified-matching C++
real-time implementation — plus a from-scratch neural-network controller
trained against the same plant and compared head-to-head with the classical
LQR baseline.

Two questions this project answers with actual numbers, not just code:

1. **Does an independent C++ reimplementation of the exact same control law
   agree with the MATLAB reference, bit-for-bit?** Yes — see
   `verification/compare_trajectories.m`. (It didn't on the first attempt;
   see `docs/DESIGN.md` for the two real bugs the cross-check caught.)
2. **Does a from-scratch, evolution-strategy-trained neural-network
   controller beat a classical LQR controller on the same aircraft, same
   cost function?** No, and here's by how much, and why that's the expected,
   theoretically-sound answer rather than a disappointing one.

## Layout

```
matlab/         nonlinear dynamics, trim, linearization, LQR design,
                NN controller + from-scratch ES training, head-to-head eval
cpp/            independent real-time C++ port of the dynamics + both
                controllers (CMake project)
verification/   the MATLAB <-> C++ parity check: exported test cases,
                exported trajectories from both sides, the diff script
docs/           design write-up, plots, training curve
screenshots/    confirmation plots
```

## Quick start

```bash
# MATLAB/Octave side (requires the Octave `control` package, or MATLAB +
# Control System Toolbox)
cd matlab
octave-cli --no-gui -q run_design_and_export.m   # trim, LQR design, nonlinear sim, exports
octave-cli --no-gui -q train_es.m                # optional: retrain the NN controller from scratch
octave-cli --no-gui -q evaluate_compare.m        # LQR vs NN head-to-head, exports NN weights + a 2nd test case

# C++ side
cd ../cpp && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make

V=../../verification
./flightctl_cpp lqr $V/aircraft_params.csv $V/case1_setup.csv -                         $V/case1_cpp_trajectory.csv
./flightctl_cpp nn  $V/aircraft_params.csv $V/case2_setup.csv $V/nn_controller_weights.csv $V/case2_cpp_trajectory.csv

# Verification
cd ../../verification
octave-cli --no-gui -q --eval "compare_trajectories"
```

See `docs/DESIGN.md` for the full technical write-up: the dynamics model,
the LQR design and its deliberate scope boundary (inner-loop only, no
heading/position hold), the NN controller architecture and training method,
the honest LQR-vs-NN result, the real-time performance numbers, and the two
real bugs the MATLAB/C++ verification step caught.
