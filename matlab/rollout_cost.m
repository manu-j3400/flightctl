function [cost, diverged, X, U] = rollout_cost(ctrl_fn, p, x_trim, delta_trim, x0, dt, n_steps, Q_reduced, R)
% ROLLOUT_COST  Simulate the nonlinear closed loop under a given control
% law and return an LQR-style quadratic cost, so the learned controller
% and the LQR controller are scored by the EXACT SAME metric (same
% Q_reduced, R used for the LQR design) -- an apples-to-apples comparison,
% not a metric invented after the fact to favor one or the other.
%
%   ctrl_fn(x) -> delta (4x1), any control law.
%
% diverged = true if the state blew up (numerical overflow / unrealistic
% attitude) before n_steps completed -- treated as a large finite
% penalty rather than NaN, so an optimizer (ES) always has a usable
% gradient-free signal even for a currently-terrible candidate policy.

idx_reduced = [4 5 6 7 8 10 11 12];

x = x0;
cost = 0;
diverged = false;

want_traj = nargout > 2;
if want_traj
    X = zeros(12, n_steps+1); U = zeros(4, n_steps+1);
    X(:,1) = x0;
end

for k = 1:n_steps
    delta = ctrl_fn(x);
    e_red = x(idx_reduced) - x_trim(idx_reduced);
    u_err = delta - delta_trim;
    cost = cost + dt * (e_red' * Q_reduced * e_red + u_err' * R * u_err);

    x = rk4_step(x, delta, p, dt);

    if want_traj
        X(:,k+1) = x; U(:,k) = delta;
    end

    if any(~isfinite(x)) || abs(x(7)) > pi || abs(x(8)) > pi/2*0.98
        diverged = true;
        cost = cost + 1e6 * (n_steps - k);   % finite penalty for the remaining horizon
        if want_traj
            X(:, k+2:end) = repmat(x, 1, n_steps-k);
        end
        break;
    end
end

if want_traj && ~diverged
    U(:, end) = ctrl_fn(x);
end

end
