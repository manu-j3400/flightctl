function [x_trim, delta_trim, info] = trim_aircraft(p, Va, gamma)
% TRIM_AIRCRAFT  Solve for a symmetric, wings-level trim condition.
%
%   [x_trim, delta_trim] = trim_aircraft(p, Va, gamma)
%
% Inputs:
%   p     - aircraft_params() struct
%   Va    - trim airspeed, m/s
%   gamma - flight-path angle, rad (0 = level flight, >0 = climbing)
%
% Finds alpha, elevator, throttle such that u_dot = w_dot = q_dot = 0 with
% beta = 0, phi = 0, p = q = r = v = w_dot... (symmetric flight, no lateral
% inputs), using the exact nonlinear dynamics in dynamics_six_dof. Lateral
% residuals (v_dot, p_dot, r_dot) are identically zero at a symmetric
% solution by construction (da = dr = 0, v = 0, p = r = 0), so only 3
% unknowns / 3 equations remain.
%
% x_trim     - 12x1 trimmed state (pn=pe=pd=0, psi=0 by convention)
% delta_trim - 4x1 trimmed control input [de; da; dr; dt]
% info       - fsolve diagnostics (residual norm, exit flag)

if nargin < 3
    gamma = 0;
end

z0 = [0.03; 0.0; 0.5];   % initial guess: [alpha; elevator; throttle]

opts = optimset('TolFun', 1e-12, 'TolX', 1e-12, 'Display', 'off');
[z_sol, fval, exitflag] = fsolve(@(z) trim_residual(z, p, Va, gamma), z0, opts);

alpha = z_sol(1); de = z_sol(2); dt = z_sol(3);
theta = alpha + gamma;

x_trim = zeros(12,1);
x_trim(4) = Va*cos(alpha);     % u
x_trim(5) = 0;                 % v
x_trim(6) = Va*sin(alpha);     % w
x_trim(8) = theta;             % theta

delta_trim = [de; 0; 0; dt];

info = struct('residual_norm', norm(fval), 'exitflag', exitflag, 'alpha', alpha);

if norm(fval) > 1e-6
    warning('trim_aircraft:poorConvergence', ...
        'Trim residual norm = %.3e; solution may not be a clean trim point.', norm(fval));
end

end

function res = trim_residual(z, p, Va, gamma)
    alpha = z(1); de = z(2); dt = z(3);
    theta = alpha + gamma;

    x = zeros(12,1);
    x(4) = Va*cos(alpha);
    x(6) = Va*sin(alpha);
    x(8) = theta;

    delta = [de; 0; 0; dt];

    xdot = dynamics_six_dof(x, delta, p);
    % udot, wdot, qdot must be zero at trim.
    res = [xdot(4); xdot(6); xdot(11)];
end
