function delta = apply_control(x, x_trim, delta_trim, K, p)
% APPLY_CONTROL  Full-state-feedback LQR control law with actuator
% saturation.
%
%   delta = apply_control(x, x_trim, delta_trim, K, p)
%
%   delta_cmd = delta_trim - K * (x - x_trim)
%   delta     = saturate(delta_cmd, limits in p)
%
% This exact law (including saturation order and limits) is reimplemented
% in cpp/src/controller.cpp for the MATLAB/C++ parity check.

delta_cmd = delta_trim - K * (x - x_trim);

de = max(-p.de_max, min(p.de_max, delta_cmd(1)));
da = max(-p.da_max, min(p.da_max, delta_cmd(2)));
dr = max(-p.dr_max, min(p.dr_max, delta_cmd(3)));
dt = max(0,         min(1,        delta_cmd(4)));

delta = [de; da; dr; dt];

end
