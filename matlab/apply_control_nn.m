function delta = apply_control_nn(x, x_trim, theta, p)
% APPLY_CONTROL_NN  Evaluate the learned NN controller at state x.
% Same reduced-state convention as the LQR path (apply_control.m):
% idx_reduced = [u v w phi theta p q r] = state indices [4 5 6 7 8 10 11 12].
% The network never sees position or heading, exactly like the LQR -- the
% comparison is apples-to-apples, same information, two different control
% laws.

idx_reduced = [4 5 6 7 8 10 11 12];
scale = nn_policy('scale');
e = (x(idx_reduced) - x_trim(idx_reduced)) ./ scale;
delta = nn_policy('forward', theta, e, p);

end
