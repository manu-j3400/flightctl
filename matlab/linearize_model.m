function [A, B] = linearize_model(x_trim, delta_trim, p)
% LINEARIZE_MODEL  Numerical Jacobian linearization of the nonlinear 6-DOF
% dynamics about a trim point.
%
%   [A, B] = linearize_model(x_trim, delta_trim, p)
%
% Returns the continuous-time state-space matrices of
%   dx/dt = A*(x - x_trim) + B*(delta - delta_trim)
% computed by central finite differences on dynamics_six_dof. Central
% differences are used (not forward) because this model's accuracy is
% cheap to afford and halves the truncation error, which matters for a
% controller design step that is sensitive to the linearization quality.

n = numel(x_trim);
m = numel(delta_trim);
A = zeros(n, n);
B = zeros(n, m);

h_x = 1e-6;
for i = 1:n
    dx = zeros(n,1); dx(i) = h_x;
    f_plus  = dynamics_six_dof(x_trim + dx, delta_trim, p);
    f_minus = dynamics_six_dof(x_trim - dx, delta_trim, p);
    A(:, i) = (f_plus - f_minus) / (2*h_x);
end

h_u = 1e-6;
for j = 1:m
    du = zeros(m,1); du(j) = h_u;
    f_plus  = dynamics_six_dof(x_trim, delta_trim + du, p);
    f_minus = dynamics_six_dof(x_trim, delta_trim - du, p);
    B(:, j) = (f_plus - f_minus) / (2*h_u);
end

end
