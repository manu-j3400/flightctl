% RUN_DESIGN_AND_EXPORT
% End-to-end script: trim -> linearize -> LQR design -> nonlinear
% closed-loop simulation -> export everything the C++ side needs to
% reproduce the identical run for the verification harness.
%
% Run with:  octave-cli --no-gui -q run_design_and_export.m
% (or from MATLAB: run_design_and_export)

clear; close all; clc;
pkg load control;

repo_root = fileparts(fileparts(mfilename('fullpath')));
verif_dir = fullfile(repo_root, 'verification');
docs_dir  = fullfile(repo_root, 'docs');
if ~exist(verif_dir, 'dir'); mkdir(verif_dir); end
if ~exist(docs_dir, 'dir'); mkdir(docs_dir); end

%% 1. Airframe + trim -------------------------------------------------------
p  = aircraft_params();
Va_trim = 15;      % m/s cruise airspeed
gamma_trim = 0;    % level flight

[x_trim, delta_trim, trim_info] = trim_aircraft(p, Va_trim, gamma_trim);
printf('Trim: alpha=%.3f deg, theta=%.3f deg, elevator=%.3f deg, throttle=%.3f (residual %.2e)\n', ...
    rad2deg(trim_info.alpha), rad2deg(x_trim(8)), rad2deg(delta_trim(1)), delta_trim(4), trim_info.residual_norm);

%% 2. Linearize ---------------------------------------------------------
[A, B] = linearize_model(x_trim, delta_trim, p);

%% 3. LQR design on the reduced 8-state inner loop -----------------------
% States order in the reduced model: [u v w phi theta p q r]
% Weight airspeed/attitude tightly, rates more loosely, controls cheaply
% relative to the (fast, light) airframe -- tuned by inspection of the
% resulting closed-loop damping/settling time below, not a first guess
% taken on faith.
Q_reduced = diag([ ...
    1.0,    ... u   (m/s)^-2 scale
    2.0,    ... v
    2.0,    ... w
    120.0,  ... phi  (rad)   -- bank angle: tight
    180.0,  ... theta (rad)  -- pitch: tight
    8.0,    ... p
    15.0,   ... q
    8.0]);  ... r
R = diag([8, 6, 6, 4]);   % [elevator, aileron, rudder, throttle]

[K, K_reduced, idx_reduced] = design_lqr(A, B, Q_reduced, R);

A_r = A(idx_reduced, idx_reduced);
B_r = B(idx_reduced, :);
cl_eig = eig(A_r - B_r*K_reduced);
printf('\nOpen-loop reduced-model eigenvalues:\n'); disp(eig(A_r)');
printf('Closed-loop reduced-model eigenvalues:\n'); disp(cl_eig');
if any(real(cl_eig) >= 0)
    warning('run_design_and_export:unstableClosedLoop', ...
        'Closed-loop system has a non-negative-real-part eigenvalue -- controller is NOT stabilizing as designed.');
else
    printf('All closed-loop eigenvalues have negative real part: locally stable about trim.\n');
end

%% 4. Nonlinear closed-loop simulation, perturbed initial condition -------
dt = 0.0025;         % 400 Hz control loop
T  = 8.0;            % seconds
N  = round(T/dt);

x0 = x_trim;
x0(7)  = x0(7)  + deg2rad(20);   % phi:   20 deg bank disturbance
x0(8)  = x0(8)  + deg2rad(15);   % theta: 15 deg pitch disturbance
x0(10) = x0(10) + deg2rad(30);   % p:     30 deg/s roll-rate disturbance
x0(11) = x0(11) + deg2rad(10);   % q:     10 deg/s pitch-rate disturbance
x0(12) = x0(12) + deg2rad(15);   % r:     15 deg/s yaw-rate disturbance

% Logging convention (must match matlab/rollout_cost.m and
% cpp/src/main.cpp exactly, or the MATLAB/C++ parity check in
% verification/compare_trajectories.m reports a spurious control-channel
% mismatch even when the dynamics are bit-exact, as happened once during
% development here): U(:,j) is ALWAYS the control law evaluated at
% X(:,j), i.e. "what the controller commands at this row's state" --
% never "the control that produced this row's state from the previous
% one". Those two conventions agree almost everywhere but drift apart by
% exactly one sample's worth of controller output, which is a small but
% real control values of its own.
X = zeros(12, N+1);
U = zeros(4, N+1);
x = x0;
for k = 1:N+1
    u = apply_control(x, x_trim, delta_trim, K, p);
    X(:, k) = x;
    U(:, k) = u;
    if k <= N
        x = rk4_step(x, u, p, dt);
    end
end
t = (0:N) * dt;

% Only phi and theta are fed back by this inner-loop regulator (psi/heading
% is deliberately NOT regulated here -- see design_lqr.m -- so a nonzero
% final psi is the documented, expected behavior of a disturbance that had
% any yaw-rate component, not a controller failure).
final_roll_pitch_err_deg = rad2deg(norm(X(7:8, end) - x_trim(7:8)));
final_heading_drift_deg  = rad2deg(X(9, end) - x_trim(9));
printf('\nFinal roll/pitch error after %.1f s: %.4f deg (regulated states)\n', T, final_roll_pitch_err_deg);
printf('Final heading drift after %.1f s: %.4f deg (NOT regulated by design -- would need an outer heading-hold loop)\n', T, final_heading_drift_deg);
if final_roll_pitch_err_deg > 1.0
    warning('run_design_and_export:notSettled', 'Roll/pitch have not settled to within 1 deg by the end of the run.');
end

%% 5. Plots ---------------------------------------------------------------
figure('Visible', 'off', 'Position', [0 0 1000 700]);

subplot(3,2,1);
plot(t, rad2deg(X(7,:)), 'LineWidth', 1.3); hold on;
plot(t, rad2deg(X(8,:)), 'LineWidth', 1.3);
plot(t, rad2deg(X(9,:)), 'LineWidth', 1.3);
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)', 'Location', 'best');
xlabel('t (s)'); ylabel('deg'); title('Euler angles'); grid on;

subplot(3,2,2);
plot(t, rad2deg(X(10,:)), 'LineWidth', 1.3); hold on;
plot(t, rad2deg(X(11,:)), 'LineWidth', 1.3);
plot(t, rad2deg(X(12,:)), 'LineWidth', 1.3);
legend('p', 'q', 'r', 'Location', 'best');
xlabel('t (s)'); ylabel('deg/s'); title('Body rates'); grid on;

subplot(3,2,3);
plot(t, X(4,:), 'LineWidth', 1.3); hold on;
plot(t, X(5,:), 'LineWidth', 1.3);
plot(t, X(6,:), 'LineWidth', 1.3);
legend('u', 'v', 'w', 'Location', 'best');
xlabel('t (s)'); ylabel('m/s'); title('Body velocity'); grid on;

subplot(3,2,4);
Va_hist = sqrt(sum(X(4:6,:).^2,1));
plot(t, Va_hist, 'LineWidth', 1.3);
xlabel('t (s)'); ylabel('m/s'); title('Airspeed'); grid on;

subplot(3,2,5);
plot(t, rad2deg(U(1,:)), 'LineWidth', 1.3); hold on;
plot(t, rad2deg(U(2,:)), 'LineWidth', 1.3);
plot(t, rad2deg(U(3,:)), 'LineWidth', 1.3);
legend('elevator', 'aileron', 'rudder', 'Location', 'best');
xlabel('t (s)'); ylabel('deg'); title('Control surfaces (saturated)'); grid on;

subplot(3,2,6);
plot(t, U(4,:), 'LineWidth', 1.3);
xlabel('t (s)'); ylabel('throttle [0,1]'); title('Throttle'); grid on;

print(fullfile(docs_dir, 'closed_loop_response.png'), '-dpng', '-r130');
printf('Saved plot: %s\n', fullfile(docs_dir, 'closed_loop_response.png'));

%% 6. Export everything the C++ side needs --------------------------------
export_params_csv(p, fullfile(verif_dir, 'aircraft_params.csv'));

fid = fopen(fullfile(verif_dir, 'case1_setup.csv'), 'w');
fprintf(fid, '# single source of truth for the verification test case.\n');
fprintf(fid, '# vectors are comma-separated on one line in the given order.\n');
fprintf(fid, 'dt,%.17g\n', dt);
fprintf(fid, 'n_steps,%d\n', N);
fprintf(fid, 'x_trim,%s\n', strjoin(arrayfun(@(v) sprintf('%.17g', v), x_trim, 'UniformOutput', false), ','));
fprintf(fid, 'delta_trim,%s\n', strjoin(arrayfun(@(v) sprintf('%.17g', v), delta_trim, 'UniformOutput', false), ','));
fprintf(fid, 'x0,%s\n', strjoin(arrayfun(@(v) sprintf('%.17g', v), x0, 'UniformOutput', false), ','));
fprintf(fid, '# K is a 4x12 row-major matrix (4 rows: elevator,aileron,rudder,throttle; 12 cols: state order as in dynamics_six_dof)\n');
for row = 1:4
    fprintf(fid, 'K_row%d,%s\n', row, strjoin(arrayfun(@(v) sprintf('%.17g', v), K(row,:), 'UniformOutput', false), ','));
end
fclose(fid);
printf('Saved setup: %s\n', fullfile(verif_dir, 'case1_setup.csv'));

fid = fopen(fullfile(verif_dir, 'case1_matlab_trajectory.csv'), 'w');
fprintf(fid, 't,pn,pe,pd,u,v,w,phi,theta,psi,p,q,r,de,da,dr,dt_thr\n');
for k = 1:(N+1)
    fprintf(fid, '%.17g,%s,%s\n', t(k), ...
        strjoin(arrayfun(@(v) sprintf('%.17g', v), X(:,k), 'UniformOutput', false), ','), ...
        strjoin(arrayfun(@(v) sprintf('%.17g', v), U(:,k), 'UniformOutput', false), ','));
end
fclose(fid);
printf('Saved trajectory: %s (%d rows)\n', fullfile(verif_dir, 'case1_matlab_trajectory.csv'), N+1);

printf('\nDone. Next: build and run the C++ side, then run verification/compare_trajectories.m\n');
