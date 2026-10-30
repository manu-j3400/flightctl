function train_es(n_generations)
% TRAIN_ES  From-scratch evolution-strategy training of the NN controller
% directly against the NONLINEAR plant (policy search, not backprop
% through the ODE -- no autodiff library needed). Resumable: call this
% repeatedly and it picks up from its own checkpoint file, so a long
% training run can be split across several invocations instead of one
% long-running call.
%
%   train_es(40)   % run 40 more generations from wherever the checkpoint is
%
% Method: (mu/mu_w, lambda)-ES.
%   - Sample lambda candidate parameter vectors around the current mean.
%   - Evaluate each on the SAME batch of randomly-drawn disturbed initial
%     conditions this generation (shared across candidates = variance
%     reduction: differences in cost reflect policy quality, not luck of
%     the draw).
%   - Rank-weighted recombination of the best mu candidates -> new mean.
%   - Track the single best candidate ever seen (elitism) separately from
%     the recombination mean, since the mean can regress even while the
%     population improves on average.

pkg load control;
repo_root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repo_root, 'matlab'));
ckpt_dir = fullfile(repo_root, 'matlab', 'checkpoints');
if ~exist(ckpt_dir, 'dir'); mkdir(ckpt_dir); end
ckpt_path = fullfile(ckpt_dir, 'nn_controller_checkpoint.mat');

%% Fixed problem setup (must match run_design_and_export.m's trim) --------
p = aircraft_params();
[x_trim, delta_trim] = trim_aircraft(p, 15, 0);

Q_reduced = diag([1.0, 2.0, 2.0, 120.0, 180.0, 8.0, 15.0, 8.0]);
R         = diag([8, 6, 6, 4]);

dt_train = 0.01;
T_train  = 3.0;
n_steps  = round(T_train/dt_train);

%% ES hyperparameters -------------------------------------------------------
lambda = 24;             % candidates per generation
mu     = 8;              % selected for recombination
episodes_per_gen = 2;    % shared initial conditions per generation
sigma0 = 0.12;
sigma_decay = 0.985;
sigma_min = 0.015;

%% Load or initialize checkpoint -------------------------------------------
if exist(ckpt_path, 'file')
    load(ckpt_path);   % brings in: theta_mean, sigma, gen, best_theta, best_cost, cost_history
    printf('Resuming from checkpoint at generation %d (best_cost so far = %.5f)\n', gen, best_cost);
else
    theta_mean = nn_policy('init', p, x_trim, delta_trim);
    sigma = sigma0;
    gen = 0;
    best_theta = theta_mean;
    best_cost = Inf;
    cost_history = [];
    printf('No checkpoint found -- starting from warm-started initialization.\n');
end

n_params = numel(theta_mean);
w_rank = log(mu+1) - log(1:mu)';
w_rank = w_rank / sum(w_rank);

old_rand_state = rand('state');
old_randn_state = randn('state');
rand('state', 1000 + gen);
randn('state', 2000 + gen);

for g = 1:n_generations
    % Shared random disturbed initial conditions for this generation.
    ics = sample_initial_conditions(episodes_per_gen, x_trim);

    candidates = theta_mean + sigma * randn(n_params, lambda);
    costs = zeros(lambda, 1);
    for i = 1:lambda
        theta_i = candidates(:, i);
        ctrl_fn = @(x) apply_control_nn(x, x_trim, theta_i, p);
        c_total = 0;
        for j = 1:episodes_per_gen
            c = rollout_cost(ctrl_fn, p, x_trim, delta_trim, ics(:,j), dt_train, n_steps, Q_reduced, R);
            c_total = c_total + c;
        end
        costs(i) = c_total / episodes_per_gen;
    end

    [sorted_costs, order] = sort(costs, 'ascend');
    top = candidates(:, order(1:mu));
    recombined = top * w_rank;

    if sorted_costs(1) < best_cost
        best_cost = sorted_costs(1);
        best_theta = candidates(:, order(1));
    end

    % Anchor the new mean half-way to the best-ever candidate. Pure
    % recombination (mean = recombined) drifted the search away from the
    % best region once sigma got small (observed: best-ever plateaued at
    % 3.70 for ~30 generations while mean(top mu) cost actually got
    % WORSE) -- a known failure mode of plain (mu,lambda)-ES without
    % elitism feeding back into the mean. This keeps exploration (via
    % `recombined`) while not letting the mean wander away from the best
    % solution found so far.
    theta_mean = 0.5*best_theta + 0.5*recombined;

    sigma = max(sigma_min, sigma * sigma_decay);
    gen = gen + 1;
    cost_history(end+1) = sorted_costs(1);

    if mod(gen, 5) == 0 || g == n_generations
        printf('gen %4d | sigma %.4f | best-this-gen %.5f | best-ever %.5f | mean(top mu) cost %.5f\n', ...
            gen, sigma, sorted_costs(1), best_cost, mean(sorted_costs(1:mu)));
    end
end

save('-mat', ckpt_path, 'theta_mean', 'sigma', 'gen', 'best_theta', 'best_cost', 'cost_history');
printf('Checkpoint saved: %s (generation %d, best_cost %.5f)\n', ckpt_path, gen, best_cost);

rand('state', old_rand_state);
randn('state', old_randn_state);

end

function ics = sample_initial_conditions(n, x_trim)
    % Training envelope: phi0 in +-25deg, theta0 in +-20deg, p0 in
    % +-35deg/s, q0 in +-20deg/s, r0 in +-25deg/s, small u/v/w jitter.
    % This DELIBERATELY covers the LQR test disturbance
    % (20,15,30,10,15 deg / deg/s) with margin, so the "within envelope"
    % comparison is fair; the separate out-of-envelope generalization
    % test in evaluate_compare.m uses a disturbance well outside this box.
    ics = zeros(12, n);
    for j = 1:n
        x0 = x_trim;
        x0(4)  = x0(4)  + 1.5*(2*rand-1);
        x0(5)  = x0(5)  + 1.5*(2*rand-1);
        x0(6)  = x0(6)  + 1.5*(2*rand-1);
        x0(7)  = x0(7)  + deg2rad(25)*(2*rand-1);
        x0(8)  = x0(8)  + deg2rad(20)*(2*rand-1);
        x0(10) = x0(10) + deg2rad(35)*(2*rand-1);
        x0(11) = x0(11) + deg2rad(20)*(2*rand-1);
        x0(12) = x0(12) + deg2rad(25)*(2*rand-1);
        ics(:,j) = x0;
    end
end
