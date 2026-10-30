function p = aircraft_params()
% AIRCRAFT_PARAMS  Physical and aerodynamic parameters of the airframe used
% throughout this project.
%
% This is a representative small fixed-wing UAV (roughly glider/trainer-class:
% ~2 m wingspan, ~2.5 kg). The mass/geometry are physically self-consistent
% (inertias computed from a simple distributed-mass assumption, see below);
% the aerodynamic stability derivatives are illustrative values chosen to be
% physically plausible for this class of airframe (positive static margin,
% stable dihedral effect, conventional control-surface authority), NOT copied
% from any specific real aircraft's flight-test or proprietary data. Swap
% these for wind-tunnel or flight-test-identified numbers to model a real
% airframe.
%
% All units SI (kg, m, s, rad) unless noted. Angles in radians.

p = struct();

%% Mass / inertia -------------------------------------------------------
p.m    = 2.5;              % kg, total mass
p.g    = 9.80665;          % m/s^2

% Geometry
p.S    = 0.55;             % m^2, wing reference area
p.b    = 2.10;             % m, wingspan
p.c    = 0.27;             % m, mean aerodynamic chord

% Inertia tensor (body axes), kg*m^2. Chosen consistent with a slender
% flying-wing-ish mass distribution: Jy > Jx, modest Jxz product-of-inertia
% from a slightly nose-down-tilted principal axis (typical for a pusher-prop
% layout). These are illustrative, not measured.
p.Jx   = 0.1147;
p.Jy   = 0.0576;
p.Jz   = 0.1712;
p.Jxz  = 0.0015;

% Precompute the Gamma coefficients used in the rotational EOM (Beard &
% McLain / standard rigid-body form with a Jxz product-of-inertia term).
Gamma    = p.Jx*p.Jz - p.Jxz^2;
p.Gamma1 = (p.Jxz*(p.Jx - p.Jy + p.Jz)) / Gamma;
p.Gamma2 = (p.Jz*(p.Jz - p.Jy) + p.Jxz^2) / Gamma;
p.Gamma3 = p.Jz / Gamma;
p.Gamma4 = p.Jxz / Gamma;
p.Gamma5 = (p.Jz - p.Jx) / p.Jy;
p.Gamma6 = p.Jxz / p.Jy;
p.Gamma7 = ((p.Jx - p.Jy)*p.Jx + p.Jxz^2) / Gamma;
p.Gamma8 = p.Jx / Gamma;

%% Atmosphere -------------------------------------------------------------
p.rho  = 1.225;            % kg/m^3, sea-level air density (no wind model here)

%% Propulsion (simple linear thrust model, body +x axis) ------------------
p.T_max = 12.0;            % N, thrust at throttle = 1

%% Longitudinal aerodynamics (wind-axis lift/drag + pitching moment) ------
% Valid for the small-perturbation, attached-flow regime around cruise this
% project trims and linearizes at (|alpha| up to a few degrees). A stall /
% separated-flow model is intentionally out of scope.
p.CL0      =  0.28;
p.CL_alpha =  4.8;         % per rad
p.CL_de    =  0.36;        % per rad elevator
p.CD0      =  0.032;
p.CD_alpha =  0.70;        % per rad^2 (parabolic drag-due-to-lift surrogate)
p.Cm0      =  0.02;
p.Cm_alpha = -0.55;        % per rad  (negative => statically stable)
p.Cm_q     = -6.4;         % per (rad/s), nondimensionalized inside dynamics
p.Cm_de    = -1.10;        % per rad elevator

%% Lateral-directional aerodynamics ---------------------------------------
p.CY_beta  = -0.30;
p.CY_dr    =  0.19;
p.Cl_beta  = -0.09;        % per rad   (negative => stable dihedral effect)
p.Cl_p     = -0.50;
p.Cl_r     =  0.07;
p.Cl_da    =  0.17;
p.Cn_beta  =  0.07;        % per rad   (positive => weathercock-stable)
p.Cn_p     = -0.03;
p.Cn_r     = -0.09;
p.Cn_da    =  0.006;
p.Cn_dr    = -0.072;

%% Control surface / throttle limits ---------------------------------------
p.de_max = deg2rad(25);
p.da_max = deg2rad(25);
p.dr_max = deg2rad(25);

end
