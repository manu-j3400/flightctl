function [K, K_reduced, idx_reduced] = design_lqr(A, B, Q_reduced, R)
% DESIGN_LQR  LQR gain for the inner-loop (velocity/attitude/rate) states.
%
%   [K, K_reduced, idx_reduced] = design_lqr(A, B, Q_reduced, R)
%
% The full 12-state linearization has 4 states with IDENTICALLY ZERO
% columns in A and B: pn, pe, pd (inertial position) and psi (heading).
% Nothing in this flat-earth, no-wind model's force/moment balance depends
% on where the aircraft is or which way it is pointed, so those 4 states
% are uncontrollable from this feedback law's point of view -- and, not
% coincidentally, they are exactly the states a real autopilot regulates
% with an OUTER guidance loop (track/altitude/heading hold commanding
% roll & pitch references) rather than a single monolithic LQR. Folding
% them into one 12-state regulator would hand the Riccati solver 4
% marginally-stable (eigenvalue-zero) uncontrollable modes, which violates
% the stabilizability condition LQR relies on.
%
% So: solve LQR on the reduced 8-state inner loop
%   idx_reduced = [u v w phi theta p q r]   (state indices [4 5 6 7 8 10 11 12])
% and return K, zero-padded back out to a 4x12 gain so callers can use it
% directly against the full 12-state error vector (x - x_trim) -- the zero
% columns simply mean "this regulator does not react to position or
% heading error", which is the deliberate, documented scope boundary of
% this project (see docs/DESIGN.md), not an oversight.

idx_reduced = [4 5 6 7 8 10 11 12];

A_r = A(idx_reduced, idx_reduced);
B_r = B(idx_reduced, :);

Co = ctrb(A_r, B_r);
if rank(Co) < size(A_r,1)
    error('design_lqr:notControllable', ...
        'Reduced 8-state pair (A_r,B_r) is not controllable (rank %d of %d). Check linearization.', ...
        rank(Co), size(A_r,1));
end

K_reduced = lqr(A_r, B_r, Q_reduced, R);

n = size(A,1);
m = size(B,2);
K = zeros(m, n);
K(:, idx_reduced) = K_reduced;

end
