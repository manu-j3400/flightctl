% EVALUATE_COMPARE
% Head-to-head comparison: LQR baseline vs. the trained NN controller,
% on (a) the SAME disturbance used in run_design_and_export.m (within the
% NN's training envelope) and (b) a disturbance deliberately well outside
% the training envelope (generalization test). Both controllers are
% scored with the identical quadratic cost (same Q_reduced, R) so "lower
% is better" means the same thing for both.

clear; close all; clc;
pkg load control;
repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repo_root, 'matlab'));
docs_dir = fullfile(repo_root, 'docs');

p = aircraft_params();
[x_trim, delta_trim] = trim_aircraft(p, 15, 0);
[A, B] = linearize_model(x_trim, delta_trim, p);
Q_reduced = diag([1.0, 2.0, 2.0, 120.0, 180.0, 8.0, 15.0, 8.0]);
R         = diag([8, 6, 6, 4]);
[K, ~, ~] = design_lqr(A, B, Q_reduced, R);

ckpt = load(fullfile(repo_root, 'matlab', 'checkpoints', 'nn_controller_checkpoint.mat'));
theta = ckpt.best_theta;
printf('Loaded NN controller: trained %d generations, training best cost %.5f\n', ckpt.gen, ckpt.best_cost);

dt = 0.0025;
T  = 8.0;
n_steps = round(T/dt);

scenarios = struct();

% (a) Within training envelope -- identical to run_design_and_export.m.
x0 = x_trim;
x0(7)  = x0(7)  + deg2rad(20);
x0(8)  = x0(8)  + deg2rad(15);
x0(10) = x0(10) + deg2rad(30);
x0(11) = x0(11) + deg2rad(10);
x0(12) = x0(12) + deg2rad(15);
scenarios(1).name = 'within_envelope';
scenarios(1).label = 'Within training envelope (20deg bank / 15deg pitch / 30-15deg/s rates)';
scenarios(1).x0 = x0;

% (b) Out-of-envelope generalization test: notably larger than anything
% the NN ever saw in training (training envelope: phi<=25, theta<=20,
% p<=35, q<=20, r<=25 deg or deg/s).
x0b = x_trim;
x0b(7)  = x0b(7)  + deg2rad(45);
x0b(8)  = x0b(8)  + deg2rad(35);
x0b(10) = x0b(10) + deg2rad(60);
x0b(11) = x0b(11) + deg2rad(40);
x0b(12) = x0b(12) + deg2rad(45);
scenarios(2).name = 'out_of_envelope';
scenarios(2).label = 'Out-of-envelope generalization test (45deg bank / 35deg pitch / 60-40deg/s rates)';
scenarios(2).x0 = x0b;

results = struct();
for s = 1:numel(scenarios)
    x0s = scenarios(s).x0;

    ctrl_lqr = @(x) apply_control(x, x_trim, delta_trim, K, p);
    [cost_lqr, div_lqr, X_lqr, U_lqr] = rollout_cost(ctrl_lqr, p, x_trim, delta_trim, x0s, dt, n_steps, Q_reduced, R);

    ctrl_nn = @(x) apply_control_nn(x, x_trim, theta, p);
    [cost_nn, div_nn, X_nn, U_nn] = rollout_cost(ctrl_nn, p, x_trim, delta_trim, x0s, dt, n_steps, Q_reduced, R);

    printf('\n=== Scenario: %s ===\n%s\n', scenarios(s).name, scenarios(s).label);
    printf('  LQR : cost = %10.4f   diverged = %d\n', cost_lqr, div_lqr);
    printf('  NN  : cost = %10.4f   diverged = %d\n', cost_nn, div_nn);
    printf('  ratio (NN/LQR) = %.3f  (>1 means LQR did better)\n', cost_nn/cost_lqr);

    results(s).scenario   = scenarios(s).name;
    results(s).cost_lqr   = cost_lqr;
    results(s).cost_nn    = cost_nn;
    results(s).diverged_lqr = div_lqr;
    results(s).diverged_nn  = div_nn;

    % Plot both on the same axes for this scenario.
    t = (0:n_steps) * dt;
    figure('Visible', 'off', 'Position', [0 0 1000 500]);
    subplot(1,2,1);
    plot(t, rad2deg(X_lqr(7,:)), 'LineWidth', 1.4); hold on;
    plot(t, rad2deg(X_nn(7,:)),  'LineWidth', 1.4, 'LineStyle', '--');
    legend('LQR \phi', 'NN \phi', 'Location', 'best');
    xlabel('t (s)'); ylabel('deg'); title(['Roll -- ' strrep(scenarios(s).name,'_',' ')]); grid on;

    subplot(1,2,2);
    plot(t, rad2deg(X_lqr(8,:)), 'LineWidth', 1.4); hold on;
    plot(t, rad2deg(X_nn(8,:)),  'LineWidth', 1.4, 'LineStyle', '--');
    legend('LQR \theta', 'NN \theta', 'Location', 'best');
    xlabel('t (s)'); ylabel('deg'); title(['Pitch -- ' strrep(scenarios(s).name,'_',' ')]); grid on;

    outfile = fullfile(docs_dir, sprintf('compare_%s.png', scenarios(s).name));
    print(outfile, '-dpng', '-r130');
    printf('  Saved plot: %s\n', outfile);
end

printf('\n=== Summary ===\n');
printf('%-18s %12s %12s %10s\n', 'scenario', 'LQR cost', 'NN cost', 'NN/LQR');
for s = 1:numel(results)
    printf('%-18s %12.4f %12.4f %10.3f\n', results(s).scenario, results(s).cost_lqr, results(s).cost_nn, results(s).cost_nn/results(s).cost_lqr);
end

%% Export the trained NN weights for the C++ port -------------------------
verif_dir = fullfile(repo_root, 'verification');
fid = fopen(fullfile(verif_dir, 'nn_controller_weights.csv'), 'w');
fprintf(fid, '# from-scratch MLP controller weights, trained by matlab/train_es.m\n');
fprintf(fid, '# layer_sizes,%s\n', strjoin(arrayfun(@(v) sprintf('%d', v), nn_policy('layers'), 'UniformOutput', false), ','));
fprintf(fid, '# scale,%s\n', strjoin(arrayfun(@(v) sprintf('%.17g', v), nn_policy('scale'), 'UniformOutput', false), ','));
fprintf(fid, '# theta,%s\n', strjoin(arrayfun(@(v) sprintf('%.17g', v), theta, 'UniformOutput', false), ','));
fclose(fid);
printf('\nExported NN weights for C++: %s\n', fullfile(verif_dir, 'nn_controller_weights.csv'));

% Also export a second verification test case (the within-envelope NN
% rollout) for the MATLAB/C++ parity harness, same format as case1.
x0 = scenarios(1).x0;
ctrl_nn = @(x) apply_control_nn(x, x_trim, theta, p);
[~, ~, X_nn, U_nn] = rollout_cost(ctrl_nn, p, x_trim, delta_trim, x0, dt, n_steps, Q_reduced, R);
t = (0:n_steps)*dt;
fid = fopen(fullfile(verif_dir, 'case2_nn_matlab_trajectory.csv'), 'w');
fprintf(fid, 't,pn,pe,pd,u,v,w,phi,theta,psi,p,q,r,de,da,dr,dt_thr\n');
for k = 1:(n_steps+1)
    fprintf(fid, '%.17g,%s,%s\n', t(k), ...
        strjoin(arrayfun(@(v) sprintf('%.17g', v), X_nn(:,k), 'UniformOutput', false), ','), ...
        strjoin(arrayfun(@(v) sprintf('%.17g', v), U_nn(:,k), 'UniformOutput', false), ','));
end
fclose(fid);
fid = fopen(fullfile(verif_dir, 'case2_setup.csv'), 'w');
fprintf(fid, 'dt,%.17g\n', dt);
fprintf(fid, 'n_steps,%d\n', n_steps);
fprintf(fid, 'x_trim,%s\n', strjoin(arrayfun(@(v) sprintf('%.17g', v), x_trim, 'UniformOutput', false), ','));
fprintf(fid, 'delta_trim,%s\n', strjoin(arrayfun(@(v) sprintf('%.17g', v), delta_trim, 'UniformOutput', false), ','));
fprintf(fid, 'x0,%s\n', strjoin(arrayfun(@(v) sprintf('%.17g', v), x0, 'UniformOutput', false), ','));
fclose(fid);
printf('Exported NN verification case: case2_setup.csv, case2_nn_matlab_trajectory.csv\n');
