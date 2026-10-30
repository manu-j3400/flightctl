function [xdot, diag_out] = dynamics_six_dof(x, delta, p)
% DYNAMICS_SIX_DOF  Nonlinear 6-DOF rigid-body fixed-wing dynamics.
%
%   xdot = dynamics_six_dof(x, delta, p)
%
% State x (12x1), NED / body convention:
%   x(1:3)  = [pn; pe; pd]   inertial (NED) position, m
%   x(4:6)  = [u;  v;  w]    body-frame velocity, m/s
%   x(7:9)  = [phi; theta; psi]   Euler angles (roll, pitch, yaw), rad
%   x(10:12)= [p; q; r]     body-frame angular rate, rad/s
%
% Input delta (4x1):
%   delta(1) = elevator deflection, rad (+ trailing edge down -> nose up... )
%   delta(2) = aileron deflection, rad
%   delta(3) = rudder deflection, rad
%   delta(4) = throttle, [0, 1]
%
% p = struct from aircraft_params().
%
% The kinematics (position rate from body velocity, Euler-angle rates from
% body rates) and the Newton-Euler rigid-body equations are the exact
% nonlinear equations (no small-angle assumption anywhere in this function).
% The only linearization in the whole model is in the aerodynamic
% coefficients themselves (CL, CD, Cm, ... as affine functions of
% alpha/beta/rates/controls) -- see aircraft_params.m.
%
% diag_out (optional second output) returns intermediate quantities
% (Va, alpha, beta, forces, moments) for logging/plotting.

u  = x(4);  v = x(5);  w = x(6);
phi = x(7); theta = x(8); psi = x(9);
pp  = x(10); q = x(11); r = x(12);

de = delta(1); da = delta(2); dr = delta(3); dt = delta(4);

%% --- Kinematics: inertial position rate from body velocity -------------
cphi = cos(phi); sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);

% Rotation matrix from body frame to NED (vehicle) frame, R_v_b = R_b_v'.
R_b_v = [ cth*cpsi,                 sphi*sth*cpsi - cphi*spsi,   cphi*sth*cpsi + sphi*spsi;
          cth*spsi,                 sphi*sth*spsi + cphi*cpsi,   cphi*sth*spsi - sphi*cpsi;
         -sth,                      sphi*cth,                    cphi*cth                   ];

pos_dot = R_b_v * [u; v; w];

%% --- Kinematics: Euler angle rates from body rates ----------------------
if abs(cth) < 1e-6
    error('dynamics_six_dof:gimbalLock', 'theta too close to +/-90 deg (gimbal lock).');
end
euler_dot = [1, sphi*tan(theta), cphi*tan(theta);
             0, cphi,            -sphi;
             0, sphi/cth,         cphi/cth] * [pp; q; r];

%% --- Airdata -------------------------------------------------------------
Va    = sqrt(u^2 + v^2 + w^2);
alpha = atan2(w, u);
if Va > 1e-6
    beta = asin(max(-1, min(1, v / Va)));
else
    beta = 0;
end
qbar = 0.5 * p.rho * Va^2;   % dynamic pressure

%% --- Aerodynamic coefficients (affine model, see aircraft_params.m) -----
CL = p.CL0 + p.CL_alpha*alpha + p.CL_de*de;
CD = p.CD0 + p.CD_alpha*alpha^2;
CY = p.CY_beta*beta + p.CY_dr*dr;

if Va > 1e-6
    p_hat = pp * p.b / (2*Va);
    q_hat = q  * p.c / (2*Va);
    r_hat = r  * p.b / (2*Va);
else
    p_hat = 0; q_hat = 0; r_hat = 0;
end

Cl_m = p.Cl_beta*beta + p.Cl_p*p_hat + p.Cl_r*r_hat + p.Cl_da*da;
Cm_m = p.Cm0 + p.Cm_alpha*alpha + p.Cm_q*q_hat + p.Cm_de*de;
Cn_m = p.Cn_beta*beta + p.Cn_p*p_hat + p.Cn_r*r_hat + p.Cn_da*da + p.Cn_dr*dr;

%% --- Forces: lift/drag/side-force (wind axes) rotated into body axes ----
L = qbar * p.S * CL;
D = qbar * p.S * CD;
Y = qbar * p.S * CY;

ca = cos(alpha); sa = sin(alpha);
fx_aero =  -D*ca + L*sa;
fz_aero =  -D*sa - L*ca;
fy_aero =   Y;

fx_grav = -p.m * p.g * sth;
fy_grav =  p.m * p.g * cth * sphi;
fz_grav =  p.m * p.g * cth * cphi;

fx_thrust = p.T_max * max(0, min(1, dt));

fx = fx_aero + fx_grav + fx_thrust;
fy = fy_aero + fy_grav;
fz = fz_aero + fz_grav;

%% --- Moments --------------------------------------------------------------
l_m = qbar * p.S * p.b * Cl_m;
m_m = qbar * p.S * p.c * Cm_m;
n_m = qbar * p.S * p.b * Cn_m;

%% --- Newton-Euler translational dynamics (body frame) --------------------
vel_dot = [ r*v - q*w + fx/p.m;
            pp*w - r*u + fy/p.m;
            q*u - pp*v + fz/p.m ];

%% --- Newton-Euler rotational dynamics (body frame, with Jxz term) --------
rate_dot = [ p.Gamma1*pp*q - p.Gamma2*q*r + p.Gamma3*l_m + p.Gamma4*n_m;
             p.Gamma5*pp*r - p.Gamma6*(pp^2 - r^2) + m_m/p.Jy;
             p.Gamma7*pp*q - p.Gamma1*q*r + p.Gamma4*l_m + p.Gamma8*n_m ];

xdot = [pos_dot; vel_dot; euler_dot; rate_dot];

if nargout > 1
    diag_out = struct('Va', Va, 'alpha', alpha, 'beta', beta, ...
                       'L', L, 'D', D, 'Y', Y, ...
                       'l', l_m, 'm', m_m, 'n', n_m, ...
                       'fx', fx, 'fy', fy, 'fz', fz);
end

end
