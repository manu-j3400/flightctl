function x_next = rk4_step(x, delta, p, dt)
% RK4_STEP  One fixed-step classical Runge-Kutta 4 integration step of
% dynamics_six_dof, holding delta constant over the step (zero-order hold,
% matching a digital control loop).
%
% This EXACT algorithm (same stage structure, same fixed dt, same
% zero-order-hold-on-input assumption) is reimplemented bit-for-bit in
% cpp/src/dynamics.cpp so that verification/compare_trajectories.m can
% check MATLAB and C++ agree on a state-by-state basis, not just "look
% similar on a plot".

k1 = dynamics_six_dof(x,            delta, p);
k2 = dynamics_six_dof(x + dt/2*k1,  delta, p);
k3 = dynamics_six_dof(x + dt/2*k2,  delta, p);
k4 = dynamics_six_dof(x + dt*k3,    delta, p);

x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

end
