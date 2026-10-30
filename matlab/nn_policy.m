function varargout = nn_policy(mode, varargin)
% NN_POLICY  A small from-scratch feedforward controller: 8 -> 24 -> 24 -> 4,
% tanh hidden activations, output layer mapped directly to actuator limits
% so the network can never command an out-of-range surface deflection.
%
% This is the "learned controller" compared head-to-head against the LQR
% baseline. No deep-learning toolbox, no autodiff, no GPU: just explicit
% matrix-vector forward passes, trained with the from-scratch evolution
% strategy in train_es.m (policy search directly on the nonlinear plant,
% not backprop through the ODE).
%
% Usage:
%   theta0       = nn_policy('init', p, x_trim, delta_trim);          % warm-started near trim
%   n            = nn_policy('nparams');
%   delta        = nn_policy('forward', theta, e, p);                 % e = normalized 8-state error (8x1 or 8xN)
%   scale        = nn_policy('scale');                                 % 8x1 input normalization vector

persistent LAYER_SIZES
if isempty(LAYER_SIZES)
    LAYER_SIZES = [8, 24, 24, 4];
end

switch mode
    case 'layers'
        varargout{1} = LAYER_SIZES;

    case 'nparams'
        varargout{1} = count_params(LAYER_SIZES);

    case 'scale'
        % Normalizes the 8 reduced states [u v w phi theta p q r] to O(1)
        % for typical disturbance magnitudes used in training/eval.
        varargout{1} = [6; 6; 6; 0.6; 0.6; 1.2; 1.2; 1.2];

    case 'init'
        p = varargin{1}; x_trim = varargin{2}; delta_trim = varargin{3};
        varargout{1} = init_warm_start(LAYER_SIZES, p, delta_trim);

    case 'forward'
        theta = varargin{1}; e = varargin{2}; p = varargin{3};
        varargout{1} = forward(theta, LAYER_SIZES, e, p);

    otherwise
        error('nn_policy:badMode', 'Unknown mode %s', mode);
end

end

%% ------------------------------------------------------------------------
function n = count_params(sizes)
    n = 0;
    for i = 1:numel(sizes)-1
        n = n + sizes(i)*sizes(i+1) + sizes(i+1);   % W + b
    end
end

function [Ws, bs] = unpack(theta, sizes)
    Ws = {}; bs = {};
    idx = 1;
    for i = 1:numel(sizes)-1
        nin = sizes(i); nout = sizes(i+1);
        Wn = nin*nout;
        Ws{i} = reshape(theta(idx:idx+Wn-1), nout, nin);
        idx = idx + Wn;
        bs{i} = theta(idx:idx+nout-1);
        idx = idx + nout;
    end
end

function theta = pack(Ws, bs)
    parts = {};
    for i = 1:numel(Ws)
        parts{end+1} = Ws{i}(:);
        parts{end+1} = bs{i}(:);
    end
    theta = vertcat(parts{:});
end

function theta0 = init_warm_start(sizes, p, delta_trim)
    rng_seed = 42; % reproducible initialization
    old_state = rand('state');
    rand('state', rng_seed);

    nlayers = numel(sizes) - 1;
    Ws = {}; bs = {};
    for i = 1:nlayers
        nin = sizes(i); nout = sizes(i+1);
        Ws{i} = 0.08 * (2*rand(nout, nin) - 1);
        bs{i} = zeros(nout, 1);
    end

    % Warm-start the OUTPUT layer bias so that, at zero input error, the
    % pre-saturation outputs land near delta_trim once the final
    % limit-mapping (see forward()) is applied. All hidden weights stay
    % small-random, so the network starts as "roughly trim, plus a small
    % nonlinear correction" instead of a random, almost-certainly-unstable
    % policy. This is what makes training from scratch tractable in a
    % modest number of generations.
    z0 = atanh_clamped([delta_trim(1)/p.de_max; delta_trim(2)/p.da_max; ...
                         delta_trim(3)/p.dr_max; 2*delta_trim(4) - 1]);
    bs{nlayers} = z0;

    theta0 = pack(Ws, bs);
    rand('state', old_state);
end

function z = atanh_clamped(x)
    x = max(-0.98, min(0.98, x));
    z = atanh(x);
end

function delta = forward(theta, sizes, e, p)
    % e: 8xN normalized error batch (N>=1)
    [Ws, bs] = unpack(theta, sizes);
    h = e;
    nlayers = numel(Ws);
    for i = 1:nlayers-1
        h = tanh(Ws{i}*h + bs{i});
    end
    z = Ws{nlayers}*h + bs{nlayers};   % 4xN pre-saturation output

    de = p.de_max * tanh(z(1,:));
    da = p.da_max * tanh(z(2,:));
    dr = p.dr_max * tanh(z(3,:));
    dt = 0.5 * (1 + tanh(z(4,:)));     % maps to [0,1]

    delta = [de; da; dr; dt];
end
